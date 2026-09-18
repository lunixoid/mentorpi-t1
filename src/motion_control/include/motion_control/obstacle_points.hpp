#ifndef MOTION_CONTROL_OBSTACLE_POINTS_HPP_
#define MOTION_CONTROL_OBSTACLE_POINTS_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace motion_control {

struct Point2d {
  double x;
  double y;
};

struct Pose2d {
  double x;
  double y;
  double yaw;
};

// Robot outline in base_footprint (SD036 D3.1): front edge at +front, back edge at -back,
// sides at +-half_width. The in-place turn centre is the frame origin.
struct Footprint {
  double front;
  double back;
  double half_width;
};

// D2.1. LaserScan rays to points in base_footprint. The lidar pose comes from TF in the node and
// is never baked in here: the URDF turns lidar_frame by pi. NaN, inf and ranges outside
// [range_min, range_max] are dropped (LD19 reports 26.0 past its range).
inline std::vector<Point2d> scan_to_base_points(const std::vector<float>& ranges, double angle_min,
                                                double angle_increment, double range_min,
                                                double range_max, const Pose2d& lidar_in_base) {
  std::vector<Point2d> points;
  points.reserve(ranges.size());
  const double c = std::cos(lidar_in_base.yaw);
  const double s = std::sin(lidar_in_base.yaw);
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double r = static_cast<double>(ranges[i]);
    if (!std::isfinite(r) || r < range_min || r > range_max) {
      continue;
    }
    const double angle = angle_min + angle_increment * static_cast<double>(i);
    const double lx = r * std::cos(angle);
    const double ly = r * std::sin(angle);
    points.push_back(Point2d{lidar_in_base.x + c * lx - s * ly, lidar_in_base.y + s * lx + c * ly});
  }
  return points;
}

// D2.2. Self hits: points inside the outline grown by pad are the robot's own shell. An object
// pressed closer than pad is invisible as well, which is why pad stays below margin_min.
inline void drop_self_points(std::vector<Point2d>& points, const Footprint& fp, double pad) {
  const auto inside = [&fp, pad](const Point2d& p) {
    return p.x <= fp.front + pad && p.x >= -(fp.back + pad) && std::abs(p.y) <= fp.half_width + pad;
  };
  points.erase(std::remove_if(points.begin(), points.end(), inside), points.end());
}

// D2.3. People are not obstacles: points within radius of any person are dropped. Returns how
// many were dropped. An empty person list drops nothing.
inline std::size_t exclude_person_points(std::vector<Point2d>& points,
                                         const std::vector<Point2d>& persons, double radius) {
  if (persons.empty()) {
    return 0;
  }
  const double radius_sq = radius * radius;
  const auto near_person = [&persons, radius_sq](const Point2d& p) {
    for (const Point2d& person : persons) {
      const double dx = p.x - person.x;
      const double dy = p.y - person.y;
      if (dx * dx + dy * dy <= radius_sq) {
        return true;
      }
    }
    return false;
  };
  const auto kept_end = std::remove_if(points.begin(), points.end(), near_person);
  const auto dropped = static_cast<std::size_t>(points.end() - kept_end);
  points.erase(kept_end, points.end());
  return dropped;
}

}  // namespace motion_control

#endif  // MOTION_CONTROL_OBSTACLE_POINTS_HPP_
