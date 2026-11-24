#include "kiss_matcher_slam/core/offline_dynamic_map_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace kiss_matcher_slam {
namespace {

struct VoxelKey {
  std::int64_t x;
  std::int64_t y;
  std::int64_t z;

  bool operator==(const VoxelKey& other) const noexcept {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& key) const noexcept {
    std::size_t seed = std::hash<std::int64_t>{}(key.x);
    seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

struct VoxelObservation {
  std::size_t count = 0;
  double first_timestamp = std::numeric_limits<double>::infinity();
  double last_timestamp = -std::numeric_limits<double>::infinity();
};

VoxelKey voxelKey(const SlamPoint& point, const double resolution) {
  return {static_cast<std::int64_t>(std::floor(point.x / resolution)),
          static_cast<std::int64_t>(std::floor(point.y / resolution)),
          static_cast<std::int64_t>(std::floor(point.z / resolution))};
}

bool finitePoint(const SlamPoint& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
         std::isfinite(point.intensity);
}

}  // namespace

OfflineDynamicMapFilter::OfflineDynamicMapFilter(OfflineDynamicMapFilterConfig config)
    : config_(config) {
  if (!std::isfinite(config_.observation_voxel_resolution) ||
      config_.observation_voxel_resolution <= 0.0 ||
      !std::isfinite(config_.output_voxel_resolution) ||
      config_.output_voxel_resolution <= 0.0 || config_.minimum_observations == 0 ||
      !std::isfinite(config_.minimum_observation_duration_s) ||
      config_.minimum_observation_duration_s < 0.0) {
    throw std::invalid_argument("invalid offline dynamic-map filter configuration");
  }
}

OfflineDynamicMapFilterResult OfflineDynamicMapFilter::filter(
    const std::vector<Keyframe>& keyframes) const {
  if (keyframes.empty()) {
    throw std::invalid_argument("cannot filter an empty keyframe sequence");
  }

  using ObservationMap = std::unordered_map<VoxelKey, VoxelObservation, VoxelKeyHash>;
  ObservationMap observations;
  OfflineDynamicMapFilterResult result;
  result.input_keyframes = keyframes.size();

  for (const auto& keyframe : keyframes) {
    if (!std::isfinite(keyframe.timestamp) || !keyframe.optimized_pose.allFinite()) {
      throw std::invalid_argument("keyframe contains a non-finite timestamp or pose");
    }
    result.input_points += keyframe.cloud.size();
    pcl::PointCloud<SlamPoint> transformed;
    pcl::transformPointCloud(keyframe.cloud, transformed, keyframe.optimized_pose);

    std::unordered_set<VoxelKey, VoxelKeyHash> frame_voxels;
    frame_voxels.reserve(transformed.size());
    for (const auto& point : transformed) {
      if (finitePoint(point)) {
        frame_voxels.insert(voxelKey(point, config_.observation_voxel_resolution));
      }
    }
    for (const auto& key : frame_voxels) {
      auto& observation = observations[key];
      ++observation.count;
      observation.first_timestamp = std::min(observation.first_timestamp, keyframe.timestamp);
      observation.last_timestamp = std::max(observation.last_timestamp, keyframe.timestamp);
    }
  }

  result.observed_voxels = observations.size();
  std::unordered_set<VoxelKey, VoxelKeyHash> persistent;
  persistent.reserve(observations.size());
  for (const auto& [key, observation] : observations) {
    const double duration = observation.last_timestamp - observation.first_timestamp;
    if (observation.count >= config_.minimum_observations &&
        duration >= config_.minimum_observation_duration_s) {
      persistent.insert(key);
    }
  }
  result.persistent_voxels = persistent.size();

  pcl::PointCloud<SlamPoint>::Ptr retained(new pcl::PointCloud<SlamPoint>());
  for (const auto& keyframe : keyframes) {
    pcl::PointCloud<SlamPoint> transformed;
    pcl::transformPointCloud(keyframe.cloud, transformed, keyframe.optimized_pose);
    for (const auto& point : transformed) {
      if (finitePoint(point) &&
          persistent.count(voxelKey(point, config_.observation_voxel_resolution)) != 0U) {
        retained->push_back(point);
      }
    }
  }
  result.retained_points_before_output_voxelization = retained->size();

  if (!retained->empty()) {
    pcl::VoxelGrid<SlamPoint> voxel_grid;
    const auto leaf = static_cast<float>(config_.output_voxel_resolution);
    voxel_grid.setLeafSize(leaf, leaf, leaf);
    voxel_grid.setInputCloud(retained);
    voxel_grid.filter(result.cloud);
  }
  return result;
}

}  // namespace kiss_matcher_slam
