#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "slam/loop_closure.h"

namespace {

kiss_matcher::LoopClosureConfig testConfig() {
  kiss_matcher::LoopClosureConfig config;
  config.enable_global_registration_ = false;
  config.is_multilayer_env_ = false;
  config.num_submap_keyframes_ = 3;
  config.voxel_res_ = 0.3;
  config.loop_detection_radius_ = 2.0;
  config.loop_detection_timediff_threshold_ = 10.0;
  config.matcher_config_ = kiss_matcher::KISSMatcherConfig(config.voxel_res_, false);
  return config;
}

kiss_matcher_slam::Keyframe makeKeyframe(const std::size_t id,
                                         const double timestamp,
                                         const double x,
                                         const double z) {
  kiss_matcher_slam::Keyframe keyframe;
  keyframe.id = id;
  keyframe.timestamp = timestamp;
  keyframe.optimized_pose(0, 3) = x;
  keyframe.optimized_pose(2, 3) = z;
  kiss_matcher_slam::SlamPoint point;
  point.x = 1.0f;
  point.y = 0.0f;
  point.z = 0.0f;
  keyframe.cloud.push_back(point);
  return keyframe;
}

TEST(LoopClosure, RejectsInvalidConfiguration) {
  auto config = testConfig();
  config.gicp_config_.max_num_iter_ = 0;
  EXPECT_THROW((void)kiss_matcher::LoopClosure(config, rclcpp::get_logger("test")),
               std::invalid_argument);

  config = testConfig();
  config.matcher_config_ = kiss_matcher::KISSMatcherConfig(0.5, false);
  EXPECT_THROW((void)kiss_matcher::LoopClosure(config, rclcpp::get_logger("test")),
               std::invalid_argument);
}

TEST(LoopClosure, SingleKeyframeCandidateSearchIsSafe) {
  const auto config = testConfig();
  kiss_matcher::LoopClosure closure(config, rclcpp::get_logger("test"));
  std::vector<kiss_matcher_slam::Keyframe> keyframes{makeKeyframe(0, 0.0, 0.0, 0.0)};
  EXPECT_TRUE(closure.getLoopCandidatesFromQuery(keyframes.front(), keyframes).empty());
}

TEST(LoopClosure, CandidateDistanceCanIgnoreOrIncludeHeight) {
  auto config = testConfig();
  std::vector<kiss_matcher_slam::Keyframe> keyframes{
      makeKeyframe(0, 0.0, 0.0, 20.0), makeKeyframe(1, 20.0, 1.0, 0.0)};

  kiss_matcher::LoopClosure planar(config, rclcpp::get_logger("test"));
  const auto planar_candidates = planar.getLoopCandidatesFromQuery(keyframes.back(), keyframes);
  ASSERT_EQ(planar_candidates.size(), 1U);
  EXPECT_EQ(planar_candidates.front().idx_, 0U);

  config.is_multilayer_env_ = true;
  kiss_matcher::LoopClosure spatial(config, rclcpp::get_logger("test"));
  EXPECT_TRUE(spatial.getLoopCandidatesFromQuery(keyframes.back(), keyframes).empty());
}

TEST(LoopClosure, SubmapBuilderValidatesIndices) {
  const auto config = testConfig();
  kiss_matcher::LoopClosure closure(config, rclcpp::get_logger("test"));
  std::vector<kiss_matcher_slam::Keyframe> keyframes{makeKeyframe(0, 0.0, 0.0, 0.0)};
  EXPECT_THROW(closure.setSrcAndTgtCloud(keyframes, 1, 0, 3, 0.3, false),
               std::invalid_argument);

  const auto [source, target] = closure.setSrcAndTgtCloud(keyframes, 0, 0, 1, 0.3, false);
  EXPECT_FALSE(source.empty());
  EXPECT_FALSE(target.empty());
}

TEST(LoopClosure, EmptyCloudRegistrationFailsSafely) {
  const auto config = testConfig();
  kiss_matcher::LoopClosure closure(config, rclcpp::get_logger("test"));
  pcl::PointCloud<PointType> empty;
  const auto result = closure.icpAlignment(empty, empty);
  EXPECT_FALSE(result.is_valid_);
  EXPECT_DOUBLE_EQ(result.overlapness_, 0.0);
  EXPECT_FALSE(closure.coarseToFineAlignment(empty, empty).is_valid_);
}

TEST(LoopClosure, FineResultPreservesGlobalInliersAndPoseOrder) {
  kiss_matcher::RegOutput fine;
  fine.is_valid_ = true;
  fine.overlapness_ = 75.0;
  fine.pose_(0, 3) = 2.0;
  Eigen::Matrix4d coarse = Eigen::Matrix4d::Identity();
  coarse(1, 3) = 3.0;
  const auto combined = kiss_matcher::combineRegistrationResults(fine, coarse, 123);
  EXPECT_TRUE(combined.is_valid_);
  EXPECT_EQ(combined.num_final_inliers_, 123U);
  EXPECT_DOUBLE_EQ(combined.overlapness_, 75.0);
  EXPECT_TRUE(combined.pose_.isApprox(fine.pose_ * coarse));
}

TEST(LoopClosure, RequiresOverlapAndFiniteTransform) {
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  EXPECT_FALSE(kiss_matcher::validFineRegistration(55.0, 55.0, pose));
  EXPECT_TRUE(kiss_matcher::validFineRegistration(90.0, 55.0, pose));
  pose(0, 3) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(kiss_matcher::validFineRegistration(90.0, 55.0, pose));
}

TEST(LoopClosure, IdenticalNondegenerateCloudCanMatch) {
  auto config = testConfig();
  config.gicp_config_.overlap_threshold_ = 55.0;
  kiss_matcher::LoopClosure closure(config, rclcpp::get_logger("test"));
  pcl::PointCloud<PointType> cloud;
  for (int x = 0; x < 12; ++x) {
    for (int y = 0; y < 12; ++y) {
      PointType point;
      point.x = x * 0.2f;
      point.y = y * 0.2f;
      point.z = static_cast<float>(0.2 * std::sin(x * 0.7) + 0.1 * std::cos(y * 0.4));
      point.intensity = 1.0f;
      cloud.push_back(point);
    }
  }
  const auto result = closure.icpAlignment(cloud, cloud);
  EXPECT_TRUE(result.is_valid_);
  EXPECT_TRUE(result.pose_.isApprox(Eigen::Matrix4d::Identity(), 0.03));
}

}  // namespace
