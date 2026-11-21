#include <gtest/gtest.h>

#include "kiss_matcher_slam/core/system_mode_manager.hpp"

namespace {

using kiss_matcher_slam::SystemMode;
using kiss_matcher_slam::SystemModeManager;

TEST(SystemModeManager, RequiresNormalAsTransitionHub) {
  SystemModeManager manager;
  EXPECT_TRUE(manager.requestEnter(SystemMode::kMapping).accepted);

  const auto direct_switch =
      manager.requestEnter(SystemMode::kRelocalization, "/maps/v1");
  EXPECT_FALSE(direct_switch.accepted);
  EXPECT_EQ(direct_switch.state.mode, SystemMode::kMapping);

  EXPECT_TRUE(manager.requestExit().accepted);
  EXPECT_EQ(manager.snapshot().mode, SystemMode::kExiting);
  EXPECT_FALSE(manager.requestEnter(SystemMode::kIncrementalMapping, "/maps/v1").accepted);

  EXPECT_TRUE(manager.completeExit(true).accepted);
  EXPECT_EQ(manager.snapshot().mode, SystemMode::kNormal);
  EXPECT_TRUE(manager.requestEnter(SystemMode::kIncrementalMapping, "/maps/v1").accepted);
}

TEST(SystemModeManager, MapBasedModesRequireMapDirectory) {
  SystemModeManager manager;
  EXPECT_FALSE(manager.requestEnter(SystemMode::kRelocalization).accepted);
  EXPECT_FALSE(manager.requestEnter(SystemMode::kIncrementalMapping).accepted);
  EXPECT_TRUE(manager.requestEnter(SystemMode::kRelocalization, "/maps/v2").accepted);
  EXPECT_EQ(manager.snapshot().active_map_directory, "/maps/v2");
}

TEST(SystemModeManager, FailedExitStillReturnsToNormalButInvalidatesLocalization) {
  SystemModeManager manager;
  manager.setLocalizationState(true, true, "/maps/v1");
  ASSERT_TRUE(manager.requestEnter(SystemMode::kMapping).accepted);
  ASSERT_TRUE(manager.requestExit().accepted);

  const auto completed = manager.completeExit(false, "map save failed");
  EXPECT_TRUE(completed.accepted);
  EXPECT_EQ(completed.state.mode, SystemMode::kNormal);
  EXPECT_FALSE(completed.state.localization_valid);
  EXPECT_EQ(completed.state.last_error, "map save failed");
}

TEST(SystemModeManager, RejectsInvalidExitLifecycleCalls) {
  SystemModeManager manager;
  EXPECT_FALSE(manager.requestExit().accepted);
  EXPECT_FALSE(manager.completeExit(true).accepted);

  ASSERT_TRUE(manager.requestEnter(SystemMode::kMapping).accepted);
  ASSERT_TRUE(manager.requestExit().accepted);
  EXPECT_FALSE(manager.requestExit().accepted);
}

}  // namespace
