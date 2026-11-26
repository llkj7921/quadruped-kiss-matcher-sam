#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <Eigen/LU>

#include "kiss_matcher_slam/core/keyframe.hpp"

namespace kiss_matcher_slam {

// A conservative visibility test. Unknown and occluded points remain usable.
struct OnlineDynamicFilterConfig {
  bool enabled = false;
  int history_frames = 8;
  int minimum_free_observations = 2;
  double angular_resolution_rad = 0.005;
  double range_margin_m = 0.5;
};

class OnlineDynamicFilter {
 public:
  explicit OnlineDynamicFilter(OnlineDynamicFilterConfig config) : config_(config) {
    if (config.history_frames < 1 || config.minimum_free_observations < 1 ||
        config.minimum_free_observations > config.history_frames ||
        !std::isfinite(config.angular_resolution_rad) || config.angular_resolution_rad < 0.001 ||
        config.angular_resolution_rad > 0.1 || !std::isfinite(config.range_margin_m) ||
        config.range_margin_m <= 0.0) throw std::invalid_argument("invalid online dynamic filter configuration");
  }

  pcl::PointCloud<SlamPoint> filter(const std::vector<Keyframe>& frames, std::size_t index) const {
    if (index >= frames.size()) throw std::out_of_range("invalid dynamic filter keyframe");
    const auto& query = frames[index];
    if (!config_.enabled || index == 0 || query.cloud.empty()) return query.cloud;
    std::vector<int> free_votes(query.cloud.size(), 0);
    std::vector<bool> occupied(query.cloud.size(), false);
    const auto begin = index > static_cast<std::size_t>(config_.history_frames)
                           ? index - config_.history_frames : 0;
    for (std::size_t i = begin; i < index; ++i) {
      const auto& reference = frames[i];
      if (!reference.optimized_pose.allFinite() || !query.optimized_pose.allFinite()) continue;
      std::unordered_map<long long, double> depths;
      for (const auto& p : reference.cloud) {
        const Eigen::Vector3d v(p.x, p.y, p.z);
        if (!v.allFinite() || v.norm() < 0.1) continue;
        const auto key = rayKey(v);
        const auto it = depths.find(key);
        if (it == depths.end()) depths.emplace(key, v.norm());
        else it->second = std::min(it->second, v.norm());
      }
      const Eigen::Matrix4d relative = reference.optimized_pose.inverse() * query.optimized_pose;
      for (std::size_t j = 0; j < query.cloud.size(); ++j) {
        const auto& p = query.cloud[j];
        const Eigen::Vector4d point(p.x, p.y, p.z, 1.0);
        if (!point.allFinite()) continue;
        const Eigen::Vector3d v = (relative * point).head<3>();
        const double range = v.norm();
        if (!v.allFinite() || range < 0.1) continue;
        const auto it = depths.find(rayKey(v));
        if (it == depths.end()) continue;
        if (it->second > range + config_.range_margin_m) ++free_votes[j];
        else if (std::abs(it->second - range) <= config_.range_margin_m) occupied[j] = true;
      }
    }
    pcl::PointCloud<SlamPoint> result;
    result.reserve(query.cloud.size());
    for (std::size_t i = 0; i < query.cloud.size(); ++i)
      if (occupied[i] || free_votes[i] < config_.minimum_free_observations)
        result.push_back(query.cloud[i]);
    return result;
  }

 private:
  long long rayKey(const Eigen::Vector3d& v) const {
    constexpr double pi = 3.14159265358979323846;
    const auto azimuth = static_cast<int>(std::floor((std::atan2(v.y(), v.x()) + pi) / config_.angular_resolution_rad));
    const auto elevation = static_cast<int>(std::floor((std::atan2(v.z(), std::hypot(v.x(), v.y())) + pi / 2.0) / config_.angular_resolution_rad));
    return static_cast<long long>(azimuth) * 10000LL + elevation;
  }
  OnlineDynamicFilterConfig config_;
};
}  // namespace kiss_matcher_slam
