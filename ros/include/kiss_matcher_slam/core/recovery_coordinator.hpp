#pragma once

#include <string>

#include "kiss_matcher_slam/core/localization_quality_monitor.hpp"

namespace kiss_matcher_slam {

enum class RecoveryState {
  kNormal,
  kSuspected,
  kRecovering,
  kRecoveredWaitingConfirmation,
  kOperatorAssistanceRequired,
};

const char* toString(RecoveryState state) noexcept;

struct RecoverySnapshot {
  RecoveryState state = RecoveryState::kNormal;
  std::string reason;
  bool motion_allowed = true;
  bool map_writes_allowed = true;
  bool autonomy_allowed = true;
};

class RecoveryCoordinator {
 public:
  explicit RecoveryCoordinator(LocalizationQualityMonitorConfig config = {});

  // Returns true when the caller should start the relocalization workflow.
  bool observeLocalizationQuality(RawLocalizationQuality quality,
                                  const std::string& reason);
  void observeLidarDegraded(const std::string& reason);
  bool beginRecovery(const std::string& reason);
  void markRelocalized();
  void requireOperatorAssistance(const std::string& reason);
  bool acknowledgeRecovery();

  bool recoveryPending() const noexcept;
  bool confirmationRequired() const noexcept;
  const RecoverySnapshot& snapshot() const noexcept { return snapshot_; }

 private:
  void setState(RecoveryState state, std::string reason = {});

  LocalizationQualityMonitor quality_monitor_;
  RecoverySnapshot snapshot_;
};

}  // namespace kiss_matcher_slam
