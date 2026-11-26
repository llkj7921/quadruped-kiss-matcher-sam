#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "kiss_matcher_slam/core/keyframe.hpp"
#include "kiss_matcher_slam/core/online_dynamic_filter.hpp"
#include "kiss_matcher_slam/core/results.hpp"

namespace kiss_matcher_slam {

class RingKeySearchIndex;

struct ScanContextConfig {
  OnlineDynamicFilterConfig dynamic_filter;
  int num_rings = 20;
  int num_sectors = 60;
  int exclude_recent = 30;
  int num_candidates = 10;
  double max_radius_m = 80.0;
  double distance_threshold = 0.18;
  double minimum_time_separation_s = 30.0;
  double sector_search_ratio = 0.1;
};

struct ScanContextDetectionStats {
  std::size_t history_size = 0;
  std::size_t descriptors_built = 0;
  std::size_t eligible_candidates = 0;
  std::size_t indexed_candidates = 0;
  std::size_t shortlisted_candidates = 0;
  std::size_t full_descriptor_comparisons = 0;
  std::size_t full_shift_evaluations = 0;
  double descriptor_build_ms = 0.0;
  double retrieval_ms = 0.0;
  double total_ms = 0.0;
};

// Stateful Scan Context place detector. Descriptor generation, storage and
// retrieval intentionally stay together because they share one configuration
// and one keyframe-indexed lifetime.
class ScanContextLoopDetector {
 public:
  explicit ScanContextLoopDetector(ScanContextConfig config);
  ~ScanContextLoopDetector();

  std::optional<PlaceCandidate> detect(const Keyframe &query,
                                       const std::vector<Keyframe> &keyframes);
  // Query a fixed, pre-built map. Unlike detect(), this does not apply temporal
  // exclusion because the query belongs to a new localization session.
  std::vector<PlaceCandidate> findCandidates(const pcl::PointCloud<SlamPoint> &query_cloud,
                                             const std::vector<Keyframe> &map_keyframes);
  std::size_t descriptorCount() const noexcept { return descriptors_.size(); }
  const ScanContextConfig &config() const noexcept { return config_; }
  const ScanContextDetectionStats &lastStats() const noexcept { return last_stats_; }

 private:
  struct DistanceResult {
    double distance = 1.0;
    int sector_shift = 0;
  };

  Eigen::MatrixXf makeDescriptor(const pcl::PointCloud<SlamPoint> &cloud) const;
  Eigen::VectorXf makeRingKey(const Eigen::MatrixXf &descriptor) const;
  Eigen::VectorXf makeSectorKey(const Eigen::MatrixXf &descriptor) const;
  int fastAlignUsingSectorKey(const Eigen::VectorXf &query,
                              const Eigen::VectorXf &candidate) const;
  DistanceResult descriptorDistance(const Eigen::MatrixXf &query,
                                    const Eigen::MatrixXf &candidate,
                                    int coarse_shift,
                                    std::size_t &shift_evaluations) const;
  void appendMissingDescriptors(const std::vector<Keyframe> &keyframes);

  ScanContextConfig config_;
  std::vector<Eigen::MatrixXf> descriptors_;
  std::vector<Eigen::VectorXf> ring_keys_;
  std::vector<Eigen::VectorXf> sector_keys_;
  std::unique_ptr<RingKeySearchIndex> ring_key_index_;
  ScanContextDetectionStats last_stats_;
};

}  // namespace kiss_matcher_slam
