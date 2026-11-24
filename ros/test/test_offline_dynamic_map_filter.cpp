#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "kiss_matcher_slam/core/offline_dynamic_map_filter.hpp"

namespace {

kiss_matcher_slam::SlamPoint point(const float x, const float y = 0.0f) {
  kiss_matcher_slam::SlamPoint value;
  value.x = x;
  value.y = y;
  value.z = 0.0f;
  value.intensity = 1.0f;
  return value;
}

kiss_matcher_slam::Keyframe frame(const std::size_t id,
                                  const double timestamp,
                                  const std::vector<kiss_matcher_slam::SlamPoint>& points) {
  kiss_matcher_slam::Keyframe keyframe;
  keyframe.id = id;
  keyframe.timestamp = timestamp;
  keyframe.cloud.insert(keyframe.cloud.end(), points.begin(), points.end());
  return keyframe;
}

TEST(OfflineDynamicMapFilter, KeepsPersistentAndRemovesTransientVoxels) {
  const std::vector<kiss_matcher_slam::Keyframe> keyframes{
      frame(0, 0.0, {point(0.05f), point(1.05f), point(2.05f)}),
      frame(1, 0.2, {point(0.06f), point(1.06f), point(3.05f)}),
      frame(2, 1.0, {point(0.04f), point(4.05f)}),
      frame(3, 2.0, {point(0.07f), point(5.05f)})};

  kiss_matcher_slam::OfflineDynamicMapFilterConfig config;
  config.observation_voxel_resolution = 0.5;
  config.output_voxel_resolution = 0.05;
  config.minimum_observations = 3;
  config.minimum_observation_duration_s = 0.5;
  const auto result = kiss_matcher_slam::OfflineDynamicMapFilter(config).filter(keyframes);

  EXPECT_EQ(result.input_keyframes, 4U);
  EXPECT_EQ(result.input_points, 10U);
  EXPECT_EQ(result.persistent_voxels, 1U);
  EXPECT_EQ(result.retained_points_before_output_voxelization, 4U);
  ASSERT_FALSE(result.cloud.empty());
  for (const auto& output : result.cloud) EXPECT_LT(output.x, 0.5f);
}

TEST(OfflineDynamicMapFilter, DurationRejectsShortBurst) {
  const std::vector<kiss_matcher_slam::Keyframe> keyframes{
      frame(0, 0.0, {point(0.05f)}), frame(1, 0.1, {point(0.06f)}),
      frame(2, 0.2, {point(0.04f)})};
  kiss_matcher_slam::OfflineDynamicMapFilterConfig config;
  config.observation_voxel_resolution = 0.5;
  config.minimum_observations = 3;
  config.minimum_observation_duration_s = 0.5;

  const auto result = kiss_matcher_slam::OfflineDynamicMapFilter(config).filter(keyframes);
  EXPECT_TRUE(result.cloud.empty());
  EXPECT_EQ(result.persistent_voxels, 0U);
}

TEST(OfflineDynamicMapFilter, RejectsInvalidConfigurationAndInput) {
  kiss_matcher_slam::OfflineDynamicMapFilterConfig config;
  config.observation_voxel_resolution = 0.0;
  EXPECT_THROW((void)kiss_matcher_slam::OfflineDynamicMapFilter(config), std::invalid_argument);

  config = {};
  config.minimum_observation_duration_s =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((void)kiss_matcher_slam::OfflineDynamicMapFilter(config), std::invalid_argument);

  const kiss_matcher_slam::OfflineDynamicMapFilter filter;
  EXPECT_THROW((void)filter.filter({}), std::invalid_argument);
}

}  // namespace
