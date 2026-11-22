#include "kiss_matcher_slam/core/frame_and_transform_contract.hpp"

#include <algorithm>
#include <cmath>

namespace kiss_matcher_slam {

bool distinctMapAndOdometryFrames(const std::string& map_frame,
                                  const std::string& odom_frame) {
  return !map_frame.empty() && !odom_frame.empty() && map_frame != odom_frame;
}

double rotationDistanceRadians(const Eigen::Matrix4d& lhs,
                               const Eigen::Matrix4d& rhs) {
  const Eigen::Matrix3d relative =
      lhs.block<3, 3>(0, 0).transpose() * rhs.block<3, 3>(0, 0);
  const double cosine = std::clamp((relative.trace() - 1.0) * 0.5, -1.0, 1.0);
  return std::acos(cosine);
}

bool transformsConsistent(const Eigen::Matrix4d& lhs,
                          const Eigen::Matrix4d& rhs,
                          const double max_translation_m,
                          const double max_rotation_rad) {
  if (max_translation_m <= 0.0 || max_rotation_rad <= 0.0) return false;
  const double translation =
      (lhs.block<3, 1>(0, 3) - rhs.block<3, 1>(0, 3)).norm();
  return translation <= max_translation_m &&
         rotationDistanceRadians(lhs, rhs) <= max_rotation_rad;
}

}  // namespace kiss_matcher_slam
