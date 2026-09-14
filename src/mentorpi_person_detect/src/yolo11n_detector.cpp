#include "mentorpi_person_detect/yolo11n_detector.hpp"

#include <ncnn/net.h>

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>

#include "mentorpi_person_detect/yolo11n_decode.hpp"

namespace mentorpi_person_detect {
namespace {

constexpr int kInputSize = 640;
constexpr int kNumClasses = 80;
constexpr float kPadValue = 114.0f;

struct LetterboxInfo {
  float scale{1.0f};
  int pad_w{0};
  int pad_h{0};
};

LetterboxInfo letterbox_params(int src_w, int src_h, int dst) {
  LetterboxInfo info;
  info.scale = std::min(static_cast<float>(dst) / static_cast<float>(src_w),
                        static_cast<float>(dst) / static_cast<float>(src_h));
  const int new_w = static_cast<int>(std::round(src_w * info.scale));
  const int new_h = static_cast<int>(std::round(src_h * info.scale));
  info.pad_w = (dst - new_w) / 2;
  info.pad_h = (dst - new_h) / 2;
  return info;
}

cv::Mat letterbox_bgr(const cv::Mat& bgr, const LetterboxInfo& info, int dst) {
  const int new_w = static_cast<int>(std::round(bgr.cols * info.scale));
  const int new_h = static_cast<int>(std::round(bgr.rows * info.scale));
  cv::Mat resized;
  cv::resize(bgr, resized, cv::Size(new_w, new_h));
  cv::Mat out(dst, dst, CV_8UC3, cv::Scalar(kPadValue, kPadValue, kPadValue));
  resized.copyTo(out(cv::Rect(info.pad_w, info.pad_h, new_w, new_h)));
  return out;
}

float box_iou(const DetectionBox& a, const DetectionBox& b) {
  const float ix1 = std::max(a.x1, b.x1);
  const float iy1 = std::max(a.y1, b.y1);
  const float ix2 = std::min(a.x2, b.x2);
  const float iy2 = std::min(a.y2, b.y2);
  const float iw = std::max(0.0f, ix2 - ix1);
  const float ih = std::max(0.0f, iy2 - iy1);
  const float inter = iw * ih;
  const float area_a = std::max(0.0f, a.x2 - a.x1) * std::max(0.0f, a.y2 - a.y1);
  const float area_b = std::max(0.0f, b.x2 - b.x1) * std::max(0.0f, b.y2 - b.y1);
  const float union_area = area_a + area_b - inter;
  if (union_area <= 0.0f) {
    return 0.0f;
  }
  return inter / union_area;
}

void nms(std::vector<DetectionBox>& boxes, float threshold) {
  std::sort(boxes.begin(), boxes.end(),
            [](const DetectionBox& a, const DetectionBox& b) { return a.score > b.score; });
  std::vector<DetectionBox> kept;
  std::vector<bool> suppressed(boxes.size(), false);
  for (std::size_t i = 0; i < boxes.size(); ++i) {
    if (suppressed[i]) {
      continue;
    }
    kept.push_back(boxes[i]);
    for (std::size_t j = i + 1; j < boxes.size(); ++j) {
      if (suppressed[j]) {
        continue;
      }
      if (box_iou(boxes[i], boxes[j]) > threshold) {
        suppressed[j] = true;
      }
    }
  }
  boxes = std::move(kept);
}

DetectionBox decode_box(float cx, float cy, float w, float h, const LetterboxInfo& info, int src_w,
                        int src_h) {
  const float x1 = (cx - w * 0.5f - static_cast<float>(info.pad_w)) / info.scale;
  const float y1 = (cy - h * 0.5f - static_cast<float>(info.pad_h)) / info.scale;
  const float x2 = (cx + w * 0.5f - static_cast<float>(info.pad_w)) / info.scale;
  const float y2 = (cy + h * 0.5f - static_cast<float>(info.pad_h)) / info.scale;
  DetectionBox box;
  box.x1 = std::max(0.0f, std::min(x1, static_cast<float>(src_w - 1)));
  box.y1 = std::max(0.0f, std::min(y1, static_cast<float>(src_h - 1)));
  box.x2 = std::max(0.0f, std::min(x2, static_cast<float>(src_w - 1)));
  box.y2 = std::max(0.0f, std::min(y2, static_cast<float>(src_h - 1)));
  return box;
}

enum class AttrLayout { kRows, kCols, kChannels };

// Ultralytics NCNN out0 is concat(box 4 + classes 80) on axis 0 → h=84, w=8400.
// Official ncnn yolov8 example instead uses h=proposals, w=84. Packed c=84,h=1,w=8400
// also appears after extract. Support all three.
float yolo_attr(const ncnn::Mat& feat, int proposal, int attr, AttrLayout layout) {
  switch (layout) {
    case AttrLayout::kRows:
      return feat.row(attr)[proposal];
    case AttrLayout::kCols:
      return feat.row(proposal)[attr];
    case AttrLayout::kChannels:
      return feat.channel(attr)[proposal];
  }
  return 0.0f;
}

}  // namespace

class Yolo11nDetector::Impl {
 public:
  ncnn::Net net;
  Yolo11nConfig config;
};

Yolo11nDetector::Yolo11nDetector() = default;

Yolo11nDetector::~Yolo11nDetector() {
  delete impl_;
  impl_ = nullptr;
}

bool Yolo11nDetector::load(const std::string& param_path, const Yolo11nConfig& config) {
  delete impl_;
  impl_ = new Impl();
  impl_->config = config;
  impl_->net.opt.num_threads = config.num_threads;
  impl_->net.opt.use_vulkan_compute = false;
  // Detect head keeps MemoryData anchors/strides alive across BinaryOps. Light
  // mode can recycle those blobs and leave WH in grid units while XY is in px.
  impl_->net.opt.lightmode = false;

  std::string bin_path = param_path;
  const std::string suffix = ".param";
  if (bin_path.size() >= suffix.size() &&
      bin_path.compare(bin_path.size() - suffix.size(), suffix.size(), suffix) == 0) {
    bin_path.replace(bin_path.size() - suffix.size(), suffix.size(), ".bin");
  } else {
    bin_path += ".bin";
  }

  if (impl_->net.load_param(param_path.c_str()) != 0) {
    delete impl_;
    impl_ = nullptr;
    loaded_ = false;
    return false;
  }
  if (impl_->net.load_model(bin_path.c_str()) != 0) {
    delete impl_;
    impl_ = nullptr;
    loaded_ = false;
    return false;
  }
  loaded_ = true;
  return true;
}

std::vector<DetectionBox> Yolo11nDetector::detect(const cv::Mat& bgr,
                                                  float confidence_threshold) const {
  std::vector<DetectionBox> out;
  if (!loaded_ || impl_ == nullptr || bgr.empty()) {
    return out;
  }

  const int input_size = impl_->config.input_size > 0 ? impl_->config.input_size : kInputSize;
  const LetterboxInfo lb = letterbox_params(bgr.cols, bgr.rows, input_size);
  const cv::Mat padded = letterbox_bgr(bgr, lb, input_size);

  ncnn::Mat in = ncnn::Mat::from_pixels(padded.data, ncnn::Mat::PIXEL_BGR2RGB, input_size,
                                        input_size, static_cast<int>(padded.step[0]));
  const float norm_vals[3] = {1.0f / 255.0f, 1.0f / 255.0f, 1.0f / 255.0f};
  in.substract_mean_normalize(nullptr, norm_vals);

  ncnn::Extractor ex = impl_->net.create_extractor();
  ex.set_light_mode(false);
  if (ex.input("in0", in) != 0) {
    return out;
  }

  ncnn::Mat output;
  if (ex.extract("out0", output) != 0) {
    return out;
  }

  ncnn::Mat feat;
  ncnn::convert_packing(output, feat, 1);

  const int num_attrs = 4 + kNumClasses;
  AttrLayout layout = AttrLayout::kRows;
  int num_proposals = 0;
  if (feat.h == num_attrs) {
    layout = AttrLayout::kRows;
    num_proposals = feat.w;
  } else if (feat.w == num_attrs) {
    layout = AttrLayout::kCols;
    num_proposals = feat.h;
  } else if (feat.c == num_attrs) {
    layout = AttrLayout::kChannels;
    num_proposals = feat.w * std::max(feat.h, 1);
  } else {
    return out;
  }

  const int person_attr = 4 + impl_->config.person_class_id;
  std::vector<DetectionBox> candidates;
  for (int i = 0; i < num_proposals; ++i) {
    const float person_score = yolo_attr(feat, i, person_attr, layout);
    if (person_score < confidence_threshold) {
      continue;
    }
    const float cx = yolo_attr(feat, i, 0, layout);
    const float cy = yolo_attr(feat, i, 1, layout);
    float bw = yolo_attr(feat, i, 2, layout);
    float bh = yolo_attr(feat, i, 3, layout);
    recover_wh_if_grid_units(cx, cy, &bw, &bh, i, input_size);
    DetectionBox box = decode_box(cx, cy, bw, bh, lb, bgr.cols, bgr.rows);
    box.score = person_score;
    box.class_id = impl_->config.person_class_id;
    if (box.x2 > box.x1 && box.y2 > box.y1) {
      candidates.push_back(box);
    }
  }

  nms(candidates, impl_->config.nms_threshold);
  return candidates;
}

}  // namespace mentorpi_person_detect
