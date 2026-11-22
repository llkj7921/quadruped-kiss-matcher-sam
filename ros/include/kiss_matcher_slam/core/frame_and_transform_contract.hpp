#pragma once

#include <string>

#include <Eigen/Core>

namespace kiss_matcher_slam {

bool distinctMapAndOdometryFrames(const std::string& map_frame,
                                  const std::string& odom_frame);
double rotationDistanceRadians(const Eigen::Matrix4d& lhs,
                               const Eigen::Matrix4d& rhs);
bool transformsConsistent(const Eigen::Matrix4d& lhs,
                          const Eigen::Matrix4d& rhs,
                          double max_translation_m,
                          double max_rotation_rad);

}  // namespace kiss_matcher_slam
