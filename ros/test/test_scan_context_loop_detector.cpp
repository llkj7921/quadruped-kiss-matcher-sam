#include <cmath>
#include <stdexcept>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "kiss_matcher_slam/core/scan_context_loop_detector.hpp"

namespace {
constexpr double kPi = 3.14159265358979323846;

kiss_matcher_slam::Keyframe makePattern(const std::size_t id,
                                        const double timestamp,
                                        const double yaw = 0.0) {
  kiss_matcher_slam::Keyframe keyframe;
  keyframe.id = id;
  keyframe.timestamp = timestamp;
  const std::vector<std::tuple<float, float, float>> polar_points{
      {2.0f, 0.0f, 1.0f}, {5.0f, 0.0f, 3.0f}, {8.0f, 0.0f, 2.0f}};
  for (const auto &[radius, angle, height] : polar_points) {
    kiss_matcher_slam::SlamPoint point;
    point.x = radius * std::cos(angle + yaw);
    point.y = radius * std::sin(angle + yaw);
    point.z = height;
    point.intensity = 1.0f;
    keyframe.cloud.push_back(point);
  }
  return keyframe;
}

kiss_matcher_slam::ScanContextConfig testConfig() {
  kiss_matcher_slam::ScanContextConfig config;
  config.num_rings = 4;
  config.num_sectors = 4;
  config.exclude_recent = 1;
  config.num_candidates = 2;
  config.max_radius_m = 10.0;
  config.distance_threshold = 0.1;
  config.minimum_time_separation_s = 20.0;
  config.sector_search_ratio = 0.1;
  return config;
}

TEST(ScanContextLoopDetector, RejectsInvalidConfiguration) {
  auto config = testConfig();
  config.num_rings = 0;
  EXPECT_THROW(kiss_matcher_slam::ScanContextLoopDetector{config}, std::invalid_argument);
  config = testConfig();
  config.distance_threshold = 1.1;
  EXPECT_THROW(kiss_matcher_slam::ScanContextLoopDetector{config}, std::invalid_argument);
  config = testConfig();
  config.minimum_time_separation_s = -1.0;
  EXPECT_THROW(kiss_matcher_slam::ScanContextLoopDetector{config}, std::invalid_argument);
  config = testConfig();
  config.sector_search_ratio = 0.0;
  EXPECT_THROW(kiss_matcher_slam::ScanContextLoopDetector{config}, std::invalid_argument);
  config = testConfig();
  config.sector_search_ratio = 1.1;
  EXPECT_THROW(kiss_matcher_slam::ScanContextLoopDetector{config}, std::invalid_argument);
}

TEST(ScanContextLoopDetector, FindsOldMatchingPlaceAndReportsYaw) {
  kiss_matcher_slam::ScanContextLoopDetector detector(testConfig());
  std::vector<kiss_matcher_slam::Keyframe> keyframes;
  keyframes.push_back(makePattern(0, 0.0));
  keyframes.push_back(makePattern(1, 10.0, kPi));
  keyframes.push_back(makePattern(2, 40.0, kPi / 2.0));

  const auto result = detector.detect(keyframes.back(), keyframes);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->keyframe_id, 0U);
  EXPECT_NEAR(result->descriptor_distance, 0.0, 1e-6);
  EXPECT_NEAR(result->yaw_offset_rad, -kPi / 2.0, 1e-6);
  EXPECT_EQ(detector.descriptorCount(), 3U);
  const auto &stats = detector.lastStats();
  EXPECT_EQ(stats.history_size, 3U);
  EXPECT_EQ(stats.descriptors_built, 3U);
  EXPECT_EQ(stats.eligible_candidates, 1U);
  EXPECT_EQ(stats.shortlisted_candidates, 1U);
  EXPECT_EQ(stats.full_descriptor_comparisons, 1U);
  EXPECT_GT(stats.full_shift_evaluations, 0U);
  EXPECT_LT(
      stats.full_shift_evaluations,
      stats.full_descriptor_comparisons * static_cast<std::size_t>(detector.config().num_sectors));
  EXPECT_GE(stats.descriptor_build_ms, 0.0);
  EXPECT_GE(stats.retrieval_ms, 0.0);
  EXPECT_GE(stats.total_ms, stats.retrieval_ms);
}

TEST(ScanContextLoopDetector, HonorsRecentAndTimeExclusions) {
  auto config = testConfig();
  config.minimum_time_separation_s = 100.0;
  kiss_matcher_slam::ScanContextLoopDetector detector(config);
  std::vector<kiss_matcher_slam::Keyframe> keyframes{
      makePattern(0, 0.0), makePattern(1, 10.0), makePattern(2, 40.0)};
  EXPECT_FALSE(detector.detect(keyframes.back(), keyframes).has_value());

  config = testConfig();
  config.exclude_recent = 3;
  kiss_matcher_slam::ScanContextLoopDetector recent_detector(config);
  EXPECT_FALSE(recent_detector.detect(keyframes.back(), keyframes).has_value());
}

TEST(ScanContextLoopDetector, RequiresContiguousKeyframeIds) {
  kiss_matcher_slam::ScanContextLoopDetector detector(testConfig());
  std::vector<kiss_matcher_slam::Keyframe> keyframes{makePattern(7, 0.0)};
  EXPECT_THROW(detector.detect(keyframes.front(), keyframes), std::invalid_argument);
}

TEST(ScanContextLoopDetector, AppendsDescriptorsIncrementally) {
  kiss_matcher_slam::ScanContextLoopDetector detector(testConfig());
  std::vector<kiss_matcher_slam::Keyframe> keyframes{makePattern(0, 0.0)};
  EXPECT_FALSE(detector.detect(keyframes.back(), keyframes).has_value());
  EXPECT_EQ(detector.descriptorCount(), 1U);

  keyframes.push_back(makePattern(1, 10.0));
  detector.detect(keyframes.back(), keyframes);
  EXPECT_EQ(detector.descriptorCount(), 2U);
}

TEST(ScanContextLoopDetector, UsesStandardLocalSectorSearchWindow) {
  auto config = testConfig();
  config.num_sectors = 60;
  config.exclude_recent = 1;
  config.num_candidates = 1;
  config.minimum_time_separation_s = 0.0;
  config.sector_search_ratio = 0.1;
  kiss_matcher_slam::ScanContextLoopDetector detector(config);

  std::vector<kiss_matcher_slam::Keyframe> keyframes{
      makePattern(0, 0.0), makePattern(1, 1.0, kPi), makePattern(2, 2.0, kPi / 2.0)};
  const auto result = detector.detect(keyframes.back(), keyframes);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->keyframe_id, 0U);
  EXPECT_EQ(detector.lastStats().full_descriptor_comparisons, 1U);
  EXPECT_EQ(detector.lastStats().full_shift_evaluations, 7U);
}

TEST(ScanContextLoopDetector, IncrementallyIndexesEveryEligibleRingKey) {
  auto config = testConfig();
  config.exclude_recent = 1;
  config.minimum_time_separation_s = 0.0;
  kiss_matcher_slam::ScanContextLoopDetector detector(config);
  std::vector<kiss_matcher_slam::Keyframe> keyframes{
      makePattern(0, 0.0), makePattern(1, 1.0), makePattern(2, 2.0)};

  detector.detect(keyframes.back(), keyframes);
  EXPECT_EQ(detector.lastStats().eligible_candidates, 1U);
  EXPECT_EQ(detector.lastStats().indexed_candidates, 1U);

  keyframes.push_back(makePattern(3, 3.0));
  detector.detect(keyframes.back(), keyframes);
  EXPECT_EQ(detector.lastStats().eligible_candidates, 2U);
  EXPECT_EQ(detector.lastStats().indexed_candidates, 2U);
}

TEST(ScanContextLoopDetector, QueriesAStaticMapWithoutTemporalExclusion) {
  auto config = testConfig();
  config.exclude_recent = 30;
  config.minimum_time_separation_s = 30.0;
  kiss_matcher_slam::ScanContextLoopDetector detector(config);
  std::vector<kiss_matcher_slam::Keyframe> reference{
      makePattern(0, 0.0), makePattern(1, 1.0, kPi), makePattern(2, 2.0, kPi / 2.0)};

  const auto candidates = detector.findCandidates(makePattern(99, 12345.0).cloud, reference);
  ASSERT_FALSE(candidates.empty());
  EXPECT_NEAR(candidates.front().descriptor_distance, 0.0, 1e-6);
  EXPECT_EQ(detector.lastStats().eligible_candidates, 3U);
}

}  // namespace
