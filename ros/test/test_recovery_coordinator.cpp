#include <gtest/gtest.h>

#include "kiss_matcher_slam/core/recovery_coordinator.hpp"

namespace kiss_matcher_slam {
namespace {

TEST(RecoveryCoordinator, DrivesRecoveryAndConfirmationWithoutStringState) {
  RecoveryCoordinator coordinator;
  EXPECT_FALSE(coordinator.observeLocalizationQuality(
      RawLocalizationQuality::kBad, "first bad frame"));
  EXPECT_TRUE(coordinator.observeLocalizationQuality(
      RawLocalizationQuality::kBad, "second bad frame"));
  EXPECT_TRUE(coordinator.recoveryPending());
  EXPECT_EQ(coordinator.snapshot().state, RecoveryState::kRecovering);
  EXPECT_FALSE(coordinator.snapshot().map_writes_allowed);

  coordinator.markRelocalized();
  EXPECT_TRUE(coordinator.confirmationRequired());
  EXPECT_EQ(coordinator.snapshot().state,
            RecoveryState::kRecoveredWaitingConfirmation);
  EXPECT_TRUE(coordinator.acknowledgeRecovery());
  EXPECT_EQ(coordinator.snapshot().state, RecoveryState::kNormal);
  EXPECT_TRUE(coordinator.snapshot().map_writes_allowed);
}

TEST(RecoveryCoordinator, KeepsAssistanceAndSuspectedTransitionsDefensive) {
  RecoveryCoordinator coordinator;
  coordinator.observeLidarDegraded("gap warning");
  EXPECT_EQ(coordinator.snapshot().state, RecoveryState::kSuspected);
  EXPECT_TRUE(coordinator.beginRecovery("gap recovery"));
  coordinator.requireOperatorAssistance("return to known area");
  EXPECT_EQ(coordinator.snapshot().state,
            RecoveryState::kOperatorAssistanceRequired);
  EXPECT_FALSE(coordinator.acknowledgeRecovery());
  EXPECT_EQ(coordinator.snapshot().reason, "return to known area");
}

}  // namespace
}  // namespace kiss_matcher_slam
