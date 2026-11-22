#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "kiss_matcher_slam/core/gps_altitude_manager.hpp"
#include "kiss_matcher_slam/core/keyframe.hpp"
#include "kiss_matcher_slam/core/keyframe_selector.hpp"
#include "kiss_matcher_slam/core/map_database.hpp"
#include "kiss_matcher_slam/core/mapping_runtime_state.hpp"
#include "kiss_matcher_slam/core/pose_graph_backend.hpp"

namespace kiss_matcher_slam {

struct KeyframeProcessorConfig {
  KeyframeSelectorConfig selection;
  std::vector<double> prior_variances;
  std::vector<double> odometry_variances;
};

struct KeyframeProcessingResult {
  bool accepted = false;
  bool initialized_session = false;
  bool loop_corrections_applied = false;
  Eigen::Matrix4d latest_optimized_pose = Eigen::Matrix4d::Identity();
  double optimization_time_ms = 0.0;
};

// Application-layer mapping transaction: keyframe selection, graph insertion,
// optimization, and atomic pose writeback are kept in one testable operation.
class KeyframeProcessor {
 public:
  KeyframeProcessor(KeyframeProcessorConfig config,
                    MapDatabase &map_database,
                    PoseGraphBackend &pose_graph_backend,
                    GpsAltitudeManager &gps_altitude_manager,
                    MappingRuntimeState &runtime_state);

  std::size_t nextKeyframeId() const noexcept;
  void restore(const std::vector<Keyframe> &keyframes);
  KeyframeProcessingResult process(const Keyframe &frame);

 private:
  KeyframeSelector selector_;
  std::vector<double> prior_variances_;
  std::vector<double> odometry_variances_;
  MapDatabase &map_database_;
  PoseGraphBackend &pose_graph_backend_;
  GpsAltitudeManager &gps_altitude_manager_;
  MappingRuntimeState &runtime_state_;
};

}  // namespace kiss_matcher_slam
