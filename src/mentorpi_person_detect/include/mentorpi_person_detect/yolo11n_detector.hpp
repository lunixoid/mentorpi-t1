#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "mentorpi_person_detect/byte_track.hpp"
#include "mentorpi_person_detect/yolo11n_decode.hpp"

namespace mentorpi_person_detect {

struct Yolo11nConfig {
  int input_size{640};
  int num_threads{2};
  float nms_threshold{0.45f};
  int person_class_id{0};
};

class Yolo11nDetector {
 public:
  Yolo11nDetector();
  ~Yolo11nDetector();

  Yolo11nDetector(const Yolo11nDetector&) = delete;
  Yolo11nDetector& operator=(const Yolo11nDetector&) = delete;

  bool load(const std::string& param_path, const Yolo11nConfig& config);

  std::vector<DetectionBox> detect(const cv::Mat& bgr, float confidence_threshold) const;

  bool loaded() const { return loaded_; }

 private:
  class Impl;
  Impl* impl_{nullptr};
  bool loaded_{false};
};

}  // namespace mentorpi_person_detect
