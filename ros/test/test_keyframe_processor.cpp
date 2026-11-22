#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include <gtsam/geometry/Pose3.h>

#include "kiss_matcher_slam/core/keyframe_processor.hpp"

namespace {

constexpr double kVariance = 1e-3;

kiss_matcher_slam::Keyframe makeFrame(const std::size_t id,
                                      const double timestamp,
                                      const double x) {
  kiss_matcher_slam::Keyframe frame;
  frame.id = id;
  frame.timestamp = timestamp;
  frame.odometry_pose(0, 3) = x;
  frame.optimized_pose = frame.odometry_pose;
  return frame;
}

struct Fixture {
  kiss_matcher_slam::MapDatabase database;
  kiss_matcher_slam::PoseGraphBackend backend;
  kiss_matcher_slam::GpsAltitudeManager gps{{false, 0.2, 1e8, 4.0, 1.345}};
  kiss_matcher_slam::MappingRuntimeState runtime;

  kiss_matcher_slam::KeyframeProcessor makeProcessor() {
    kiss_matcher_slam::KeyframeProcessorConfig config;
    config.selection = {1.0, 0.2, 0.0};
    config.prior_variances = std::vector<double>(6, kVariance);
    config.odometry_variances = std::vector<double>(6, kVariance);
    return {config, database, backend, gps, runtime};
  }
};

TEST(KeyframeProcessor, InitializesThenRejectsInsufficientMotion) {
  Fixture fixture;
  auto processor = fixture.makeProcessor();

  const auto initial = processor.process(makeFrame(0, 1.0, 0.0));
  EXPECT_TRUE(initial.accepted);
  EXPECT_TRUE(initial.initialized_session);
  EXPECT_EQ(processor.nextKeyframeId(), 1U);
  EXPECT_EQ(fixture.database.size(), 1U);

  const auto ignored = processor.process(makeFrame(1, 2.0, 0.5));
  EXPECT_FALSE(ignored.accepted);
  EXPECT_EQ(processor.nextKeyframeId(), 1U);
  EXPECT_EQ(fixture.database.size(), 1U);
}

TEST(KeyframeProcessor, AddsOdometryAndOptimizesAcceptedKeyframe) {
  Fixture fixture;
  auto processor = fixture.makeProcessor();
  processor.process(makeFrame(0, 1.0, 0.0));

  const auto accepted = processor.process(makeFrame(1, 2.0, 1.5));
  EXPECT_TRUE(accepted.accepted);
  EXPECT_FALSE(accepted.initialized_session);
  EXPECT_FALSE(accepted.loop_corrections_applied);
  EXPECT_EQ(fixture.database.size(), 2U);
  EXPECT_EQ(fixture.backend.estimates().size(), 2U);
  EXPECT_NEAR(accepted.latest_optimized_pose(0, 3), 1.5, 1e-6);
}

TEST(KeyframeProcessor, RejectsOutOfSequenceFrame) {
  Fixture fixture;
  auto processor = fixture.makeProcessor();
  EXPECT_THROW(processor.process(makeFrame(3, 1.0, 0.0)), std::invalid_argument);
  EXPECT_TRUE(fixture.database.empty());
}

TEST(KeyframeProcessor, RestoresMapAndContinuesKeyframeSequence) {
  Fixture fixture;
  auto processor = fixture.makeProcessor();
  const std::vector<kiss_matcher_slam::Keyframe> restored{
      makeFrame(0, 1.0, 0.0), makeFrame(1, 2.0, 1.5)};

  processor.restore(restored);
  EXPECT_EQ(processor.nextKeyframeId(), 2U);
  EXPECT_EQ(fixture.database.size(), 2U);
  EXPECT_EQ(fixture.backend.estimates().size(), 2U);
  EXPECT_TRUE(fixture.runtime.consumeMapRebuildRequest());
  EXPECT_TRUE(fixture.runtime.consumeGraphRefreshRequest());

  const auto continued = processor.process(makeFrame(2, 3.0, 3.0));
  EXPECT_TRUE(continued.accepted);
  EXPECT_EQ(fixture.database.size(), 3U);
  EXPECT_EQ(fixture.backend.estimates().size(), 3U);
}

TEST(KeyframeProcessor, RejectsRestoreAfterSessionInitialization) {
  Fixture fixture;
  auto processor = fixture.makeProcessor();
  processor.process(makeFrame(0, 1.0, 0.0));
  EXPECT_THROW(processor.restore({makeFrame(0, 1.0, 0.0)}), std::logic_error);
}

TEST(KeyframeProcessor, AppliesLoopCorrectionDuringNextMappingTransaction) {
  Fixture fixture;
  auto processor = fixture.makeProcessor();
  processor.process(makeFrame(0, 1.0, 0.0));
  processor.process(makeFrame(1, 2.0, 1.5));

  const std::vector<double> variances(6, kVariance);
  fixture.backend.addLoop(
      1,
      0,
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-1.5, 0.0, 0.0)),
      variances,
      1.345);
  const auto corrected = processor.process(makeFrame(2, 3.0, 3.0));

  EXPECT_TRUE(corrected.loop_corrections_applied);
  EXPECT_TRUE(fixture.runtime.consumeMapRebuildRequest());
  EXPECT_TRUE(fixture.runtime.consumeGraphRefreshRequest());
  EXPECT_EQ(fixture.database.size(), 3U);
  EXPECT_TRUE(fixture.database.latest().optimized_pose.allFinite());
}

TEST(KeyframeProcessor, RejectsInvalidNoiseDimensions) {
  Fixture fixture;
  kiss_matcher_slam::KeyframeProcessorConfig config;
  config.prior_variances = {1.0};
  config.odometry_variances = std::vector<double>(6, kVariance);
  EXPECT_THROW(kiss_matcher_slam::KeyframeProcessor(
                   config, fixture.database, fixture.backend, fixture.gps, fixture.runtime),
               std::invalid_argument);
}

TEST(KeyframeProcessor, RejectsNonPositiveNoise) {
  Fixture fixture;
  kiss_matcher_slam::KeyframeProcessorConfig config;
  config.prior_variances = std::vector<double>(6, kVariance);
  config.odometry_variances = std::vector<double>(6, kVariance);
  config.odometry_variances[2] = 0.0;
  EXPECT_THROW(kiss_matcher_slam::KeyframeProcessor(
                   config, fixture.database, fixture.backend, fixture.gps, fixture.runtime),
               std::invalid_argument);
}

}  // namespace
