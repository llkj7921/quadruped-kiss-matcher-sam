#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>
#include "kiss_matcher_slam/core/keyframe.hpp"

namespace kiss_matcher_slam {

struct VisibilityMapFilterConfig {
  double angular_resolution_rad = 0.01;
  double range_margin_m = 0.4;
  double maximum_range_m = 30.0;
  double minimum_time_separation_s = 0.5;
  double output_voxel_resolution_m = 0.1;
  std::size_t minimum_free_observations = 3;
  double free_to_occupied_ratio = 2.0;
  // Off by default: avoids storing per-return evidence in existing callers.
  bool capture_point_evidence = false;
};

struct VisibilityPointEvidence {
  std::uint32_t free = 0;
  std::uint32_t occupied = 0;
  std::uint32_t occluded = 0;
};

struct VisibilityMapFilterResult {
  pcl::PointCloud<SlamPoint> retained;
  pcl::PointCloud<SlamPoint> removed;
  pcl::PointCloud<SlamPoint> uncertain;
  std::size_t input_points = 0;
  std::size_t static_points = 0;
  std::size_t dynamic_points = 0;
  std::size_t uncertain_points = 0;
  std::size_t free_observations = 0;
  std::size_t occupied_observations = 0;
  std::size_t occluded_observations = 0;
  std::vector<std::vector<VisibilityPointEvidence>> point_evidence;
};

// Approximate visibility using measured returns in a small angular cone.
// Missing, occluded and ambiguous evidence never proves free space.
class VisibilityMapFilter {
 public:
  explicit VisibilityMapFilter(VisibilityMapFilterConfig config = {});
  VisibilityMapFilterResult filter(const std::vector<Keyframe>& keyframes) const;
 private:
  VisibilityMapFilterConfig config_;
};
}  // namespace kiss_matcher_slam
