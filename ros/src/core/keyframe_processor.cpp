#include "kiss_matcher_slam/core/keyframe_processor.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

namespace kiss_matcher_slam {
namespace {

gtsam::Pose3 toGtsam(const Eigen::Matrix4d &pose) {
  return {gtsam::Rot3(pose.block<3, 3>(0, 0)),
          gtsam::Point3(pose(0, 3), pose(1, 3), pose(2, 3))};
}

Eigen::Matrix4d toEigen(const gtsam::Pose3 &pose) {
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  result.block<3, 3>(0, 0) = pose.rotation().matrix();
  result.block<3, 1>(0, 3) = pose.translation();
  return result;
}

}  // namespace

KeyframeProcessor::KeyframeProcessor(KeyframeProcessorConfig config,
                                     MapDatabase &map_database,
                                     PoseGraphBackend &pose_graph_backend,
                                     GpsAltitudeManager &gps_altitude_manager,
                                     MappingRuntimeState &runtime_state)
    : selector_(config.selection),
      prior_variances_(std::move(config.prior_variances)),
      odometry_variances_(std::move(config.odometry_variances)),
      map_database_(map_database),
      pose_graph_backend_(pose_graph_backend),
      gps_altitude_manager_(gps_altitude_manager),
      runtime_state_(runtime_state) {
  const auto valid_variances = [](const std::vector<double> &variances) {
    if (variances.size() != 6) return false;
    for (const double variance : variances) {
      if (!std::isfinite(variance) || variance <= 0.0) return false;
    }
    return true;
  };
  if (!valid_variances(prior_variances_) || !valid_variances(odometry_variances_)) {
    throw std::invalid_argument(
        "pose graph prior and odometry variances must contain 6 positive finite values");
  }
}

std::size_t KeyframeProcessor::nextKeyframeId() const noexcept {
  return runtime_state_.nextKeyframeId();
}

void KeyframeProcessor::restore(const std::vector<Keyframe> &keyframes) {
  if (keyframes.empty()) throw std::invalid_argument("cannot restore an empty map product");
  if (!map_database_.empty() || runtime_state_.initialized() ||
      !pose_graph_backend_.estimates().empty()) {
    throw std::logic_error("mapping session is already initialized");
  }

  for (std::size_t id = 0; id < keyframes.size(); ++id) {
    auto frame = keyframes[id];
    if (frame.id != id) throw std::invalid_argument("restored keyframe IDs must be contiguous");
    frame.place_recognition_processed = true;
    frame.proximity_search_processed = true;
    map_database_.append(frame);
    const auto pose = toGtsam(frame.optimized_pose);
    if (id == 0) {
      pose_graph_backend_.addPrior(id, pose, prior_variances_);
    } else {
      const auto previous = toGtsam(keyframes[id - 1].optimized_pose);
      pose_graph_backend_.addOdometry(
          id - 1, id, previous.between(pose), pose, odometry_variances_);
    }
  }
  const auto optimization = pose_graph_backend_.optimize();
  if (optimization.estimates.size() != keyframes.size()) {
    throw std::runtime_error("restored pose graph size does not match map product");
  }
  runtime_state_.reset(keyframes.size());
  runtime_state_.requestMapRebuild();
  runtime_state_.requestGraphRefresh();
}

KeyframeProcessingResult KeyframeProcessor::process(const Keyframe &frame) {
  KeyframeProcessingResult result;
  const auto expected_id = runtime_state_.nextKeyframeId();
  if (frame.id != expected_id) {
    throw std::invalid_argument("incoming keyframe id does not match the mapping session sequence");
  }

  if (!runtime_state_.initialized()) {
    map_database_.append(frame);
    pose_graph_backend_.addPrior(frame.id, toGtsam(frame.odometry_pose), prior_variances_);
    runtime_state_.commitKeyframe();
    result.accepted = true;
    result.initialized_session = true;
    result.latest_optimized_pose = frame.optimized_pose;
    return result;
  }

  const auto previous = map_database_.latest();
  if (!selector_.shouldCreate(frame.optimized_pose,
                              frame.timestamp,
                              previous.optimized_pose,
                              previous.timestamp)) {
    return result;
  }

  map_database_.append(frame);
  const auto previous_pose = toGtsam(previous.optimized_pose);
  const auto current_pose = toGtsam(frame.optimized_pose);
  pose_graph_backend_.addOdometry(
      previous.id, frame.id, previous_pose.between(current_pose), current_pose, odometry_variances_);

  const auto gps_constraint =
      gps_altitude_manager_.constraint(frame.timestamp, frame.optimized_pose);
  if (gps_constraint) {
    pose_graph_backend_.addGps(frame.id,
                               gtsam::Point3(gps_constraint->position.x(),
                                             gps_constraint->position.y(),
                                             gps_constraint->position.z()),
                               gps_constraint->variances,
                               gps_constraint->huber_k);
  }
  runtime_state_.commitKeyframe();

  const auto optimization_start = std::chrono::steady_clock::now();
  const auto optimization = pose_graph_backend_.optimize();
  result.optimization_time_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                optimization_start)
          .count();
  if (optimization.estimates.empty()) {
    throw std::runtime_error("pose graph optimization returned no estimates");
  }
  result.latest_optimized_pose =
      toEigen(optimization.estimates.at<gtsam::Pose3>(optimization.estimates.size() - 1));
  result.loop_corrections_applied = optimization.loop_constraints_applied;

  if (result.loop_corrections_applied) {
    std::vector<MapDatabase::PoseUpdate> updates;
    updates.reserve(optimization.estimates.size());
    for (std::size_t id = 0; id < optimization.estimates.size(); ++id) {
      updates.emplace_back(id, toEigen(optimization.estimates.at<gtsam::Pose3>(id)));
    }
    if (!map_database_.updateOptimizedPoses(updates)) {
      throw std::runtime_error("pose graph result does not match the map database");
    }
    runtime_state_.requestMapRebuild();
    runtime_state_.requestGraphRefresh();
  }

  result.accepted = true;
  return result;
}

}  // namespace kiss_matcher_slam
