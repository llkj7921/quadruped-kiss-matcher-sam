#pragma once

#include <mutex>
#include <string>

namespace kiss_matcher_slam {

enum class SystemMode {
  kNormal,
  kMapping,
  kIncrementalMapping,
  kRelocalization,
  kExiting,
};

const char* toString(SystemMode mode) noexcept;

struct ModeSnapshot {
  SystemMode mode = SystemMode::kNormal;
  SystemMode exiting_from = SystemMode::kNormal;
  bool map_loaded = false;
  bool localization_valid = false;
  std::string active_map_directory;
  std::string last_error;
};

struct ModeTransitionResult {
  bool accepted = false;
  ModeSnapshot state;
  std::string message;
};

// Owns the system-level mode transition policy. Business modes may only be
// entered from NORMAL and must complete EXITING before another mode can start.
class SystemModeManager {
 public:
  ModeSnapshot snapshot() const;

  ModeTransitionResult requestEnter(SystemMode target,
                                    const std::string& map_directory = {});
  ModeTransitionResult requestExit();
  ModeTransitionResult completeExit(bool success, const std::string& error = {});

  void setLocalizationState(bool map_loaded,
                            bool localization_valid,
                            const std::string& map_directory = {});

 private:
  ModeTransitionResult resultLocked(bool accepted, const std::string& message) const;

  mutable std::mutex mutex_;
  ModeSnapshot state_;
};

}  // namespace kiss_matcher_slam
