#ifndef MENTORPI_PERCEPTION_PERSON_GEOMETRY_HPP_
#define MENTORPI_PERCEPTION_PERSON_GEOMETRY_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace mentorpi_perception {

// Aurora holes are zeros at the origin (SD012: is_dense, holes as zeros).
constexpr float kCloudOriginEpsM = 1.0e-4f;

// Nearest-cluster slab: the person is closer than the wall behind, so the near
// face of the bbox wins. Depth spread of a standing body plus Aurora noise.
constexpr float kClusterSlabM = 0.35f;

// Support the near cluster must have before it becomes a target. Guards against
// a single speckle in front of the person (SD022 D6.3).
constexpr int kClusterMinPoints = 40;

struct Xyz {
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
};

struct PersonXyRange {
  float x{0.0f};
  float y{0.0f};
  float range{0.0f};
};

// Pinhole intrinsics of the RGB frame (CameraInfo k). Distortion is not applied:
// Aurora's d is ~1e-2 and does not move the transverse sign (SD022 I10).
struct CameraIntrinsics {
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  int width{0};
  int height{0};
};

// Flat xyz float32 view of a PointCloud2 payload.
// /aurora/points2 is NOT organized: it is a compacted list of valid points with
// the holes dropped, so there is no pixel->slot mapping in any traversal order
// (SD022 D6.1, measured on the stand). Points are matched to pixels by
// projection only.
struct CloudXyzView {
  const std::uint8_t* data{nullptr};
  std::size_t data_bytes{0};
  std::size_t point_count{0};
  int point_step{0};
  int offset_x{0};
  int offset_y{0};
  int offset_z{0};
};

// Pixel bounds of a 2D bbox (image y down).
struct BboxPx {
  double u_min{0.0};
  double u_max{0.0};
  double v_min{0.0};
  double v_max{0.0};
};

inline bool cloud_point_valid(const Xyz& p) {
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
    return false;
  }
  return std::hypot(p.x, p.y, p.z) > kCloudOriginEpsM;
}

// A default-constructed CameraIntrinsics has fx == 0, so "no CameraInfo yet"
// and "unusable CameraInfo" are the same answer here (SD022 D6.4).
inline bool intrinsics_valid(const CameraIntrinsics& cam) {
  return std::isfinite(cam.fx) && std::isfinite(cam.fy) && std::isfinite(cam.cx) &&
         std::isfinite(cam.cy) && cam.fx > 0.0 && cam.fy > 0.0;
}

// Optical xyz -> RGB pixel. The cloud is registered to the RGB frame: on the
// stand this reproduces the picture with 0.99 colour correlation (SD022 D6.2).
inline bool project_point(const CameraIntrinsics& cam, const Xyz& p, double* u_px, double* v_px) {
  if (u_px == nullptr || v_px == nullptr || !intrinsics_valid(cam)) {
    return false;
  }
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || p.z <= 0.0f) {
    return false;
  }
  const double z = static_cast<double>(p.z);
  *u_px = cam.fx * (static_cast<double>(p.x) / z) + cam.cx;
  *v_px = cam.fy * (static_cast<double>(p.y) / z) + cam.cy;
  return std::isfinite(*u_px) && std::isfinite(*v_px);
}

// Detection2D centre/size -> pixel bounds.
inline bool bbox_from_detection(double center_x, double center_y, double size_x, double size_y,
                                BboxPx* out) {
  if (out == nullptr) {
    return false;
  }
  if (!std::isfinite(center_x) || !std::isfinite(center_y) || !std::isfinite(size_x) ||
      !std::isfinite(size_y)) {
    return false;
  }
  if (size_x <= 0.0 || size_y <= 0.0) {
    return false;
  }
  out->u_min = center_x - (size_x * 0.5);
  out->u_max = center_x + (size_x * 0.5);
  out->v_min = center_y - (size_y * 0.5);
  out->v_max = center_y + (size_y * 0.5);
  return true;
}

inline bool bbox_contains(const BboxPx& box, double u_px, double v_px) {
  return u_px >= box.u_min && u_px <= box.u_max && v_px >= box.v_min && v_px <= box.v_max;
}

// SD030 D1.3: rotate vertical bbox edges in the optical plane. When the robot
// turns left (+yaw), scene content shifts right in the image.
inline BboxPx rotate_bbox_yaw(const BboxPx& box, const CameraIntrinsics& cam, double dyaw_rad) {
  if (dyaw_rad == 0.0) {
    return box;
  }
  if (!intrinsics_valid(cam)) {
    return box;
  }
  BboxPx out = box;
  out.u_min = cam.cx + cam.fx * std::tan(std::atan((box.u_min - cam.cx) / cam.fx) + dyaw_rad);
  out.u_max = cam.cx + cam.fx * std::tan(std::atan((box.u_max - cam.cx) / cam.fx) + dyaw_rad);
  if (!std::isfinite(out.u_min) || !std::isfinite(out.u_max) || !std::isfinite(out.v_min) ||
      !std::isfinite(out.v_max)) {
    return box;
  }
  return out;
}

// SD030 D1.5: clip bbox to the RGB frame; false when there is no overlap.
inline bool bbox_clip_to_image(const BboxPx& box, const CameraIntrinsics& cam, BboxPx* out) {
  if (out == nullptr || cam.width <= 0 || cam.height <= 0) {
    return false;
  }
  const double u_lo = 0.0;
  const double u_hi = static_cast<double>(cam.width - 1);
  const double v_lo = 0.0;
  const double v_hi = static_cast<double>(cam.height - 1);
  out->u_min = std::max(box.u_min, u_lo);
  out->u_max = std::min(box.u_max, u_hi);
  out->v_min = std::max(box.v_min, v_lo);
  out->v_max = std::min(box.v_max, v_hi);
  if (out->u_min >= out->u_max || out->v_min >= out->v_max) {
    return false;
  }
  return true;
}

inline bool read_cloud_xyz(const CloudXyzView& cloud, std::size_t index, Xyz* out) {
  if (out == nullptr || cloud.data == nullptr || cloud.point_step < 4) {
    return false;
  }
  if (index >= cloud.point_count) {
    return false;
  }
  const int max_xyz_end = std::max(cloud.offset_x, std::max(cloud.offset_y, cloud.offset_z)) + 4;
  if (max_xyz_end > cloud.point_step) {
    return false;
  }
  const std::size_t base = index * static_cast<std::size_t>(cloud.point_step);
  if (base + static_cast<std::size_t>(max_xyz_end) > cloud.data_bytes) {
    return false;
  }
  Xyz p;
  std::memcpy(&p.x, cloud.data + base + cloud.offset_x, sizeof(float));
  std::memcpy(&p.y, cloud.data + base + cloud.offset_y, sizeof(float));
  std::memcpy(&p.z, cloud.data + base + cloud.offset_z, sizeof(float));
  *out = p;
  return true;
}

inline float median_of(std::vector<float>* values) {
  if (values == nullptr || values->empty()) {
    return 0.0f;
  }
  const std::size_t mid = values->size() / 2;
  std::nth_element(values->begin(), values->begin() + static_cast<std::ptrdiff_t>(mid),
                   values->end());
  return (*values)[mid];
}

// Nearest cluster of cloud points projecting inside the bbox. The person is in
// front of the background, so the near slab is the target; the median of the
// slab is robust to Aurora's holes along the silhouette (SD022 D6.3).
// Returns false when nothing projects in, or when the near slab has less than
// min_points support — no target beats a wrong target.
inline bool nearest_cluster_in_bbox(const CloudXyzView& cloud, const CameraIntrinsics& cam,
                                    const BboxPx& box, float slab_m, int min_points, Xyz* out) {
  if (out == nullptr || !intrinsics_valid(cam) || cloud.data == nullptr) {
    return false;
  }
  if (!(slab_m > 0.0f) || min_points < 1) {
    return false;
  }
  std::vector<Xyz> inside;
  float z_near = 0.0f;
  bool have_near = false;
  for (std::size_t i = 0; i < cloud.point_count; ++i) {
    Xyz p;
    if (!read_cloud_xyz(cloud, i, &p)) {
      break;
    }
    if (!cloud_point_valid(p)) {
      continue;
    }
    double u_px = 0.0;
    double v_px = 0.0;
    if (!project_point(cam, p, &u_px, &v_px)) {
      continue;
    }
    if (!bbox_contains(box, u_px, v_px)) {
      continue;
    }
    inside.push_back(p);
    if (!have_near || p.z < z_near) {
      z_near = p.z;
      have_near = true;
    }
  }
  if (!have_near) {
    return false;
  }
  // Walk the near face outwards: the first slab with enough support is the
  // person. A speckle in front of them is skipped, not taken as the target.
  std::vector<float> xs;
  std::vector<float> ys;
  std::vector<float> zs;
  const std::size_t needed = static_cast<std::size_t>(min_points);
  while (true) {
    const float z_far = z_near + slab_m;
    xs.clear();
    ys.clear();
    zs.clear();
    float next_z = 0.0f;
    bool have_next = false;
    for (const Xyz& p : inside) {
      if (p.z < z_near) {
        continue;
      }
      if (p.z <= z_far) {
        xs.push_back(p.x);
        ys.push_back(p.y);
        zs.push_back(p.z);
        continue;
      }
      if (!have_next || p.z < next_z) {
        next_z = p.z;
        have_next = true;
      }
    }
    if (xs.size() >= needed) {
      out->x = median_of(&xs);
      out->y = median_of(&ys);
      out->z = median_of(&zs);
      return true;
    }
    if (!have_next) {
      return false;
    }
    z_near = next_z;
  }
}

// Horizontal range in base_footprint. D2.1, D2.3.
inline PersonXyRange xy_range_in_base(double x, double y) {
  PersonXyRange out;
  out.x = static_cast<float>(x);
  out.y = static_cast<float>(y);
  out.range = static_cast<float>(std::hypot(x, y));
  return out;
}

}  // namespace mentorpi_perception

#endif  // MENTORPI_PERCEPTION_PERSON_GEOMETRY_HPP_
