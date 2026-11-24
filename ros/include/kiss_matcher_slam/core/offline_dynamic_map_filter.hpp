#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <pcl/point_cloud.h>

#include "kiss_matcher_slam/core/keyframe.hpp"

namespace kiss_matcher_slam {

struct OfflineDynamicMapFilterConfig {
  double observation_voxel_resolution = 0.3;
  double output_voxel_resolution = 0.1;
  std::size_t minimum_observations = 3;
  double minimum_observation_duration_s = 0.5;
};

struct OfflineDynamicMapFilterResult {
  pcl::PointCloud<SlamPoint> cloud;
  std::size_t input_keyframes = 0;
  std::size_t input_points = 0;
  std::size_t observed_voxels = 0;
  std::size_t persistent_voxels = 0;
  std::size_t retained_points_before_output_voxelization = 0;
};

// Removes short-lived map voxels using observations from distinct keyframes.
// This is a geometric persistence filter, not a semantic human detector.
class OfflineDynamicMapFilter {
 public:
  explicit OfflineDynamicMapFilter(OfflineDynamicMapFilterConfig config = {});

  OfflineDynamicMapFilterResult filter(const std::vector<Keyframe>& keyframes) const;

  const OfflineDynamicMapFilterConfig& config() const noexcept { return config_; }

 private:
  OfflineDynamicMapFilterConfig config_;
};

}  // namespace kiss_matcher_slam
