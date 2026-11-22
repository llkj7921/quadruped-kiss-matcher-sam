#include <cmath>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "kiss_matcher_slam/core/frame_and_transform_contract.hpp"

TEST(FrameContract, RequiresDistinctNonEmptyFrames) {
  EXPECT_TRUE(kiss_matcher_slam::distinctMapAndOdometryFrames("map", "odom"));
  EXPECT_FALSE(kiss_matcher_slam::distinctMapAndOdometryFrames("map", "map"));
  EXPECT_FALSE(kiss_matcher_slam::distinctMapAndOdometryFrames("", "odom"));
}

TEST(FrameContract, RelocalizationInputsMustBothUseConfiguredOdometryFrame) {
  EXPECT_TRUE(kiss_matcher_slam::relocalizationInputFramesValid(
      "odom", "odom", "odom"));
  EXPECT_FALSE(kiss_matcher_slam::relocalizationInputFramesValid(
      "map", "odom", "odom"));
  EXPECT_FALSE(kiss_matcher_slam::relocalizationInputFramesValid(
      "odom", "lidar", "odom"));
  EXPECT_FALSE(kiss_matcher_slam::relocalizationInputFramesValid(
      "odom", "odom", ""));
}

TEST(TransformContract, RejectsRotationDisagreementEvenWhenTranslationMatches) {
  Eigen::Matrix4d first = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d second = Eigen::Matrix4d::Identity();
  second.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(20.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_FALSE(kiss_matcher_slam::transformsConsistent(
      first, second, 1.0, 10.0 * M_PI / 180.0));
  EXPECT_TRUE(kiss_matcher_slam::transformsConsistent(
      first, second, 1.0, 25.0 * M_PI / 180.0));
}

TEST(TransformContract, RequiresTranslationAndRotationAgreement) {
  Eigen::Matrix4d first = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d second = Eigen::Matrix4d::Identity();
  second(0, 3) = 1.1;
  EXPECT_FALSE(kiss_matcher_slam::transformsConsistent(first, second, 1.0, 0.2));
}
