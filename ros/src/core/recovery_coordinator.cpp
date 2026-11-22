#include "kiss_matcher_slam/core/recovery_coordinator.hpp"

#include <utility>

namespace kiss_matcher_slam {

const char* toString(const RecoveryState state) noexcept {
  switch (state) {
    case RecoveryState::kNormal:
      return "normal";
    case RecoveryState::kSuspected:
      return "suspected";
    case RecoveryState::kRecovering:
      return "recovering";
    case RecoveryState::kRecoveredWaitingConfirmation:
      return "recovered_waiting_confirmation";
    case RecoveryState::kOperatorAssistanceRequired:
      return "operator_assistance_required";
  }
  return "unknown";
}

RecoveryCoordinator::RecoveryCoordinator(const LocalizationQualityMonitorConfig config)
    : quality_monitor_(config) {}

bool RecoveryCoordinator::observeLocalizationQuality(
    const RawLocalizationQuality quality, const std::string& reason) {
  if (snapshot_.state != RecoveryState::kNormal &&
      snapshot_.state != RecoveryState::kSuspected) {
    return false;
  }
  const auto health = quality_monitor_.update(quality);
  if (health == LocalizationHealth::kBad) return beginRecovery(reason);
  if (health == LocalizationHealth::kSuspected) {
    setState(RecoveryState::kSuspected, reason);
  } else if (snapshot_.state == RecoveryState::kSuspected) {
    setState(RecoveryState::kNormal);
  }
  return false;
}

void RecoveryCoordinator::observeLidarDegraded(const std::string& reason) {
  if (snapshot_.state != RecoveryState::kNormal) return;
  quality_monitor_.markSuspected();
  setState(RecoveryState::kSuspected, reason);
}

bool RecoveryCoordinator::beginRecovery(const std::string& reason) {
  if (recoveryPending() || confirmationRequired()) return false;
  setState(RecoveryState::kRecovering, reason);
  return true;
}

void RecoveryCoordinator::markRelocalized() {
  if (recoveryPending()) setState(RecoveryState::kRecoveredWaitingConfirmation);
}

void RecoveryCoordinator::requireOperatorAssistance(const std::string& reason) {
  if (recoveryPending()) setState(RecoveryState::kOperatorAssistanceRequired, reason);
}

bool RecoveryCoordinator::acknowledgeRecovery() {
  if (!confirmationRequired()) return false;
  quality_monitor_.reset();
  setState(RecoveryState::kNormal);
  return true;
}

bool RecoveryCoordinator::recoveryPending() const noexcept {
  return snapshot_.state == RecoveryState::kRecovering ||
         snapshot_.state == RecoveryState::kOperatorAssistanceRequired;
}

bool RecoveryCoordinator::confirmationRequired() const noexcept {
  return snapshot_.state == RecoveryState::kRecoveredWaitingConfirmation;
}

void RecoveryCoordinator::setState(const RecoveryState state, std::string reason) {
  snapshot_.state = state;
  snapshot_.reason = std::move(reason);
  snapshot_.motion_allowed = true;
  const bool normal = state == RecoveryState::kNormal;
  snapshot_.map_writes_allowed = normal;
  snapshot_.autonomy_allowed = normal;
}

}  // namespace kiss_matcher_slam
