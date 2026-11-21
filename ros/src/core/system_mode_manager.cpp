#include "kiss_matcher_slam/core/system_mode_manager.hpp"

namespace kiss_matcher_slam {

const char* toString(const SystemMode mode) noexcept {
  switch (mode) {
    case SystemMode::kNormal:
      return "normal";
    case SystemMode::kMapping:
      return "mapping";
    case SystemMode::kIncrementalMapping:
      return "incremental_mapping";
    case SystemMode::kRelocalization:
      return "relocalization";
    case SystemMode::kExiting:
      return "exiting";
  }
  return "unknown";
}

ModeSnapshot SystemModeManager::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

ModeTransitionResult SystemModeManager::requestEnter(
    const SystemMode target, const std::string& map_directory) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (target == SystemMode::kNormal || target == SystemMode::kExiting) {
    return resultLocked(false, "target is not an enterable business mode");
  }
  if (state_.mode != SystemMode::kNormal) {
    return resultLocked(false,
                        "exit the current mode to normal before entering another mode");
  }
  if ((target == SystemMode::kIncrementalMapping ||
       target == SystemMode::kRelocalization) &&
      map_directory.empty()) {
    return resultLocked(false, "the requested mode requires a map directory");
  }

  state_.mode = target;
  state_.exiting_from = SystemMode::kNormal;
  state_.last_error.clear();
  if (!map_directory.empty()) state_.active_map_directory = map_directory;
  if (target == SystemMode::kRelocalization) state_.localization_valid = false;
  return resultLocked(true, std::string("entered ") + toString(target));
}

ModeTransitionResult SystemModeManager::requestExit() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_.mode == SystemMode::kNormal) {
    return resultLocked(false, "already in normal mode");
  }
  if (state_.mode == SystemMode::kExiting) {
    return resultLocked(false, "mode exit is already in progress");
  }

  state_.exiting_from = state_.mode;
  state_.mode = SystemMode::kExiting;
  return resultLocked(true,
                      std::string("exiting ") + toString(state_.exiting_from));
}

ModeTransitionResult SystemModeManager::completeExit(const bool success,
                                                     const std::string& error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_.mode != SystemMode::kExiting) {
    return resultLocked(false, "no mode exit is in progress");
  }

  const auto completed_mode = state_.exiting_from;
  state_.mode = SystemMode::kNormal;
  state_.exiting_from = SystemMode::kNormal;
  state_.last_error = success ? std::string{} : error;
  if (!success) state_.localization_valid = false;
  return resultLocked(true,
                      std::string(success ? "completed " : "failed ") +
                          toString(completed_mode) + "; returned to normal");
}

void SystemModeManager::setLocalizationState(const bool map_loaded,
                                             const bool localization_valid,
                                             const std::string& map_directory) {
  std::lock_guard<std::mutex> lock(mutex_);
  state_.map_loaded = map_loaded;
  state_.localization_valid = localization_valid;
  if (!map_directory.empty()) state_.active_map_directory = map_directory;
}

ModeTransitionResult SystemModeManager::resultLocked(
    const bool accepted, const std::string& message) const {
  ModeTransitionResult result;
  result.accepted = accepted;
  result.state = state_;
  result.message = message;
  return result;
}

}  // namespace kiss_matcher_slam
