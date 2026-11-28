#include <gtest/gtest.h>
#include "kiss_matcher_slam/core/relocalization_confirmation.hpp"

TEST(RelocalizationConfirmation, NoCandidateBreaksConsecutiveSuccesses) {
  kiss_matcher_slam::RelocalizationConfirmation confirmation;
  const auto identity = Eigen::Matrix4d::Identity().eval();
  EXPECT_EQ(confirmation.observe(3, identity, 1.0, 0.2), 1);
  confirmation.reset();  // no candidate or all geometric matches failed
  EXPECT_EQ(confirmation.observe(3, identity, 1.0, 0.2), 1);
  EXPECT_EQ(confirmation.observe(3, identity, 1.0, 0.2), 2);
}

TEST(RelocalizationConfirmation, CandidateOrPoseChangeRestartsCount) {
  kiss_matcher_slam::RelocalizationConfirmation confirmation;
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  EXPECT_EQ(confirmation.observe(3, pose, 1.0, 0.2), 1);
  EXPECT_EQ(confirmation.observe(4, pose, 1.0, 0.2), 1);
  pose(0, 3) = 2.0;
  EXPECT_EQ(confirmation.observe(4, pose, 1.0, 0.2), 1);
}
