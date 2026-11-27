#include <gtest/gtest.h>
#include "kiss_matcher_slam/core/visibility_map_filter.hpp"

using namespace kiss_matcher_slam;
namespace {
Keyframe frame(double time, std::initializer_list<float> ranges) {
  Keyframe f; f.timestamp = time;
  for (float x : ranges) { SlamPoint p; p.x=x; p.y=0; p.z=0; p.intensity=1; f.cloud.push_back(p); }
  return f;
}
TEST(VisibilityMapFilter, RemovesPointWithRepeatedFreeSpaceEvidence) {
  const auto result = VisibilityMapFilter().filter({frame(0,{5}),frame(1,{10}),frame(2,{10}),frame(3,{10})});
  EXPECT_EQ(result.dynamic_points, 1U);
  ASSERT_EQ(result.removed.size(), 1U);
  EXPECT_FLOAT_EQ(result.removed[0].x, 5.0f);
  EXPECT_EQ(result.static_points, 3U);
}
TEST(VisibilityMapFilter, RetainsOccludedAndUnknownPoints) {
  const auto result = VisibilityMapFilter().filter({frame(0,{10,7}),frame(1,{3}),frame(2,{3}),frame(3,{3})});
  EXPECT_GE(result.occluded_observations, 2U);
  EXPECT_GE(result.uncertain_points, 2U);
  for (const auto& p : result.removed) EXPECT_LT(p.x, 4.0f);
}
TEST(VisibilityMapFilter, OccupiedEvidenceCanPreventRemoval) {
  const auto result = VisibilityMapFilter().filter({frame(0,{5}),frame(1,{10}),frame(2,{10}),frame(3,{10}),frame(4,{5}),frame(5,{5})});
  EXPECT_EQ(result.dynamic_points, 0U);
  EXPECT_GT(result.occupied_observations, 0U);
}
TEST(VisibilityMapFilter, HandlesSensorPoseAndRejectsInvalidConfiguration) {
  auto first = frame(0,{5});
  auto r1 = frame(1,{11}); auto r2 = frame(2,{11}); auto r3 = frame(3,{11});
  r1.optimized_pose(0,3) = -1; r2.optimized_pose(0,3) = -1; r3.optimized_pose(0,3) = -1;
  const auto result = VisibilityMapFilter().filter({first,r1,r2,r3});
  EXPECT_EQ(result.dynamic_points, 1U);
  VisibilityMapFilterConfig config; config.angular_resolution_rad = 0;
  EXPECT_THROW((void)VisibilityMapFilter(config), std::invalid_argument);
  EXPECT_THROW((void)VisibilityMapFilter().filter({}), std::invalid_argument);
}
TEST(VisibilityMapFilter, CapturesExactFramePointVotesOnlyWhenRequested) {
  const std::vector<Keyframe> frames{frame(0,{5}),frame(1,{10}),frame(2,{10}),frame(3,{10})};
  const auto original = VisibilityMapFilter().filter(frames);
  EXPECT_TRUE(original.point_evidence.empty());
  VisibilityMapFilterConfig config; config.capture_point_evidence = true;
  const auto captured = VisibilityMapFilter(config).filter(frames);
  ASSERT_EQ(captured.point_evidence.size(), frames.size());
  ASSERT_EQ(captured.point_evidence[0].size(), 1U);
  EXPECT_EQ(captured.point_evidence[0][0].free, 3U);
  EXPECT_EQ(captured.point_evidence[0][0].occupied, 0U);
  EXPECT_EQ(captured.dynamic_points, original.dynamic_points);
  EXPECT_EQ(captured.static_points, original.static_points);
  EXPECT_EQ(captured.uncertain_points, original.uncertain_points);
}
}  // namespace
