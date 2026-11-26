#include <gtest/gtest.h>
#include "kiss_matcher_slam/core/online_dynamic_filter.hpp"

using namespace kiss_matcher_slam;

namespace {
Keyframe frame(std::initializer_list<float> ranges) {
  Keyframe f;
  for (float range : ranges) {
    SlamPoint p;
    p.x = range; p.y = 0.0f; p.z = 0.0f; p.intensity = 1.0f;
    f.cloud.push_back(p);
  }
  return f;
}

TEST(OnlineDynamicFilter, RemovesOnlyWithMultipleFreeSpaceObservations) {
  OnlineDynamicFilterConfig config;
  config.enabled = true;
  std::vector<Keyframe> frames{frame({10}), frame({10}), frame({5, 10})};
  const auto result = OnlineDynamicFilter(config).filter(frames, 2);
  ASSERT_EQ(result.size(), 1U);
  EXPECT_FLOAT_EQ(result[0].x, 10.0f);
  EXPECT_EQ(frames[2].cloud.size(), 2U);
}

TEST(OnlineDynamicFilter, PreservesUnknownOccludedAndSupportedPoints) {
  OnlineDynamicFilterConfig config;
  config.enabled = true;
  const OnlineDynamicFilter filter(config);
  EXPECT_EQ(filter.filter({frame({10}), frame({5})}, 1).size(), 1U);
  EXPECT_EQ(filter.filter({frame({3}), frame({3}), frame({5})}, 2).size(), 1U);
  EXPECT_EQ(filter.filter({frame({10}), frame({10}), frame({5}), frame({5})}, 3).size(), 1U);
  EXPECT_EQ(filter.filter({frame({})}, 0).size(), 0U);
}

TEST(OnlineDynamicFilter, HandlesPoseAndDisabledMode) {
  OnlineDynamicFilterConfig config;
  config.enabled = true;
  auto first = frame({11});
  auto second = frame({11});
  auto query = frame({4, 9});
  query.optimized_pose(0, 3) = 2.0;
  const std::vector<Keyframe> frames{first, second, query};
  const auto result = OnlineDynamicFilter(config).filter(frames, 2);
  ASSERT_EQ(result.size(), 1U);
  EXPECT_FLOAT_EQ(result[0].x, 9.0f);
  config.enabled = false;
  EXPECT_EQ(OnlineDynamicFilter(config).filter(frames, 2).size(), 2U);
  config.minimum_free_observations = 9;
  EXPECT_THROW((void)OnlineDynamicFilter(config), std::invalid_argument);
}
}  // namespace
