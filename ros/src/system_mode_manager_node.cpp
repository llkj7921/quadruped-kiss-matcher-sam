#include <chrono>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <slam_system_interfaces/msg/localization_health.hpp>
#include <slam_system_interfaces/msg/lidar_stream_status.hpp>
#include <slam_system_interfaces/msg/mapping_mode_result.hpp>
#include <slam_system_interfaces/msg/recovery_status.hpp>
#include <slam_system_interfaces/msg/relocalization_status.hpp>
#include <slam_system_interfaces/msg/system_mode.hpp>

#include "kiss_matcher_slam/core/system_mode_manager.hpp"
#include "kiss_matcher_slam/core/localization_quality_monitor.hpp"

namespace kiss_matcher_slam {

class SystemModeManagerNode final : public rclcpp::Node {
 public:
  using Trigger = std_srvs::srv::Trigger;

  SystemModeManagerNode() : Node("system_mode_manager") {
    map_directory_ = declare_parameter<std::string>("map_directory", "");
    state_pub_ = create_publisher<std_msgs::msg::String>(
        "system_mode", rclcpp::QoS(1).reliable().transient_local());
    state_typed_pub_ = create_publisher<slam_system_interfaces::msg::SystemMode>(
        "/system_mode/state", rclcpp::QoS(1).reliable().transient_local());
    mapping_result_sub_ =
        create_subscription<slam_system_interfaces::msg::MappingModeResult>(
        "/mapping/mode_state", rclcpp::QoS(1).reliable().transient_local(),
        [this](const slam_system_interfaces::msg::MappingModeResult::ConstSharedPtr message) {
          if (message->state == slam_system_interfaces::msg::MappingModeResult::ERROR) {
            if (manager_.snapshot().mode == SystemMode::kExiting) {
              manager_.completeExit(false, message->error);
              publishState();
            } else {
              failActiveMode(message->error);
            }
          } else if (message->state ==
                         slam_system_interfaces::msg::MappingModeResult::READY_EXITED &&
                     manager_.snapshot().mode == SystemMode::kExiting) {
            manager_.completeExit(true);
            publishState();
            if (recovery_pending_) enterRecoveryRelocalization();
          }
        });
    lidar_recovery_sub_ =
        create_subscription<slam_system_interfaces::msg::LidarStreamStatus>(
        "/lidar/stream_status", rclcpp::QoS(1).reliable().transient_local(),
        [this](const slam_system_interfaces::msg::LidarStreamStatus::ConstSharedPtr message) {
          std::ostringstream details;
          details << message->reason << " gap_s=" << message->gap_seconds;
          if (message->state == slam_system_interfaces::msg::LidarStreamStatus::RECOVERING) {
            beginRecovery(details.str());
          } else if (message->state ==
                         slam_system_interfaces::msg::LidarStreamStatus::DEGRADED &&
                     !recovery_pending_) {
            quality_monitor_.markSuspected();
            recovery_status_ = "suspected";
            recovery_reason_ = details.str();
            publishRecoveryState();
          }
        });
    localization_health_sub_ =
        create_subscription<slam_system_interfaces::msg::LocalizationHealth>(
            "/localization/health", rclcpp::QoS(1).reliable().transient_local(),
            [this](const slam_system_interfaces::msg::LocalizationHealth::ConstSharedPtr message) {
              handleLocalizationHealth(*message);
            });
    relocalization_recovery_sub_ =
        create_subscription<slam_system_interfaces::msg::RelocalizationStatus>(
        "/relocalization/status", rclcpp::QoS(1).reliable().transient_local(),
        [this](const slam_system_interfaces::msg::RelocalizationStatus::ConstSharedPtr message) {
          if (message->state == slam_system_interfaces::msg::RelocalizationStatus::LOCALIZED) {
            completeRelocalization();
          } else if (recovery_pending_ && message->state ==
              slam_system_interfaces::msg::RelocalizationStatus::OPERATOR_ASSISTANCE_REQUIRED) {
            recovery_status_ = "operator_assistance_required";
            recovery_reason_ = message->reason;
            publishRecoveryState();
          }
        });
    recovery_pub_ = create_publisher<std_msgs::msg::String>(
        "/recovery/status", rclcpp::QoS(1).reliable().transient_local());
    recovery_typed_pub_ = create_publisher<slam_system_interfaces::msg::RecoveryStatus>(
        "/recovery/state", rclcpp::QoS(1).reliable().transient_local());

    enter_mapping_ = create_service<std_srvs::srv::Trigger>(
        "mode/enter_mapping",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          if (recovery_confirmation_required_) {
            rejectForRecovery(response);
          } else {
            respond(manager_.requestEnter(SystemMode::kMapping), response);
          }
        });
    enter_incremental_mapping_ = create_service<std_srvs::srv::Trigger>(
        "mode/enter_incremental_mapping",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          if (recovery_confirmation_required_) {
            rejectForRecovery(response);
          } else {
            respond(manager_.requestEnter(SystemMode::kIncrementalMapping, mapDirectory()), response);
          }
        });
    enter_relocalization_ = create_service<std_srvs::srv::Trigger>(
        "mode/enter_relocalization",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          respond(manager_.requestEnter(SystemMode::kRelocalization, mapDirectory()), response);
        });
    exit_mode_ = create_service<std_srvs::srv::Trigger>(
        "mode/exit",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          respond(manager_.requestExit(), response);
        });
    complete_exit_ = create_service<std_srvs::srv::Trigger>(
        "mode/complete_exit",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          respond(manager_.completeExit(true), response);
        });
    status_ = create_service<std_srvs::srv::Trigger>(
        "mode/status",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          response->success = true;
          response->message = describe(manager_.snapshot());
          publishState();
        });
    recovery_status_service_ = create_service<std_srvs::srv::Trigger>(
        "recovery/status",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          response->success = recovery_status_ == "normal";
          response->message = recoveryDescription();
        });
    recovery_ack_service_ = create_service<std_srvs::srv::Trigger>(
        "recovery/acknowledge",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          if (!recovery_confirmation_required_) {
            response->success = false;
            response->message = "no recovered localization is awaiting confirmation";
            return;
          }
          recovery_confirmation_required_ = false;
          recovery_pending_ = false;
          recovery_status_ = "normal";
          recovery_reason_.clear();
          quality_monitor_.reset();
          publishRecoveryState();
          response->success = true;
          response->message = recoveryDescription();
        });

    publishState();
    publishRecoveryState();
  }

 private:
  std::string mapDirectory() {
    get_parameter("map_directory", map_directory_);
    return map_directory_;
  }

  static std::string describe(const ModeSnapshot& state) {
    std::ostringstream stream;
    stream << "mode=" << toString(state.mode)
           << " map_loaded=" << (state.map_loaded ? "true" : "false")
           << " localization_valid=" << (state.localization_valid ? "true" : "false");
    if (!state.active_map_directory.empty()) {
      stream << " map_directory=" << state.active_map_directory;
    }
    if (!state.last_error.empty()) stream << " error=" << state.last_error;
    return stream.str();
  }

  void respond(const ModeTransitionResult& result,
               const std::shared_ptr<Trigger::Response>& response) {
    response->success = result.accepted;
    response->message = result.message + "; " + describe(result.state);
    if (result.accepted) {
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    }
    publishState();
  }

  void publishState() {
    const auto snapshot = manager_.snapshot();
    std_msgs::msg::String message;
    message.data = describe(snapshot);
    state_pub_->publish(message);
    slam_system_interfaces::msg::SystemMode typed;
    typed.mode = static_cast<uint8_t>(snapshot.mode);
    typed.map_loaded = snapshot.map_loaded;
    typed.localization_valid = snapshot.localization_valid;
    typed.map_directory = snapshot.active_map_directory;
    typed.error = snapshot.last_error;
    state_typed_pub_->publish(typed);
  }

  void completeRelocalization() {
    if (manager_.snapshot().mode != SystemMode::kRelocalization) return;

    manager_.setLocalizationState(true, true, mapDirectory());
    if (recovery_pending_) {
      recovery_status_ = "recovered_waiting_confirmation";
      recovery_confirmation_required_ = true;
      publishRecoveryState();
    }
    const auto exiting = manager_.requestExit();
    if (!exiting.accepted) return;
    RCLCPP_INFO(get_logger(), "relocalization confirmed; returning to normal localization");
    publishState();

    exit_completion_timer_ = create_wall_timer(
        std::chrono::milliseconds(100), [this]() {
          exit_completion_timer_->cancel();
          const auto completed = manager_.completeExit(true);
          if (!completed.accepted) {
            RCLCPP_ERROR(get_logger(), "%s", completed.message.c_str());
          }
          publishState();
        });
  }

  void beginRecovery(const std::string& reason) {
    if (recovery_pending_) return;
    recovery_pending_ = true;
    recovery_confirmation_required_ = false;
    recovery_status_ = "recovering";
    recovery_reason_ = reason;
    publishRecoveryState();

    const auto mode = manager_.snapshot().mode;
    if (mode == SystemMode::kNormal) {
      enterRecoveryRelocalization();
    } else if (mode == SystemMode::kMapping ||
               mode == SystemMode::kIncrementalMapping) {
      const auto exiting = manager_.requestExit();
      if (exiting.accepted) publishState();
    }
  }

  void handleLocalizationHealth(
      const slam_system_interfaces::msg::LocalizationHealth& report) {
    RawLocalizationQuality raw;
    if (report.state == slam_system_interfaces::msg::LocalizationHealth::GOOD) {
      raw = RawLocalizationQuality::kGood;
    } else if (report.state == slam_system_interfaces::msg::LocalizationHealth::SUSPECTED) {
      raw = RawLocalizationQuality::kSuspected;
    } else if (report.state == slam_system_interfaces::msg::LocalizationHealth::BAD) {
      raw = RawLocalizationQuality::kBad;
    } else {
      RCLCPP_WARN(get_logger(), "Ignoring invalid localization health state: %u", report.state);
      return;
    }
    std::ostringstream details;
    details << "typed health=" << static_cast<int>(report.state)
            << " effective_features=" << report.effective_features
            << " mean_residual_m=" << report.mean_residual_m
            << " queue_delay_ms=" << report.queue_delay_ms;
    if (!report.reason.empty()) details << " reason=" << report.reason;

    const auto health = quality_monitor_.update(raw);
    if (recovery_pending_ || recovery_confirmation_required_) return;
    if (health == LocalizationHealth::kBad) {
      beginRecovery(details.str());
    } else if (health == LocalizationHealth::kSuspected) {
      recovery_status_ = "suspected";
      recovery_reason_ = details.str();
      publishRecoveryState();
    } else if (recovery_status_ == "suspected") {
      recovery_status_ = "normal";
      recovery_reason_.clear();
      publishRecoveryState();
    }
  }

  void enterRecoveryRelocalization() {
    const auto entered =
        manager_.requestEnter(SystemMode::kRelocalization, mapDirectory());
    if (!entered.accepted) {
      recovery_status_ = "operator_assistance_required";
      recovery_reason_ = entered.message;
      publishRecoveryState();
    }
    publishState();
  }

  void rejectForRecovery(const std::shared_ptr<Trigger::Response>& response) {
    response->success = false;
    response->message =
        "operator confirmation is required before starting mapping; " +
        recoveryDescription();
  }

  std::string recoveryDescription() const {
    const bool recovered = recovery_status_ == "normal";
    const bool map_writes = recovered && !recovery_confirmation_required_;
    const bool autonomy = recovered && !recovery_confirmation_required_;
    std::ostringstream stream;
    stream << "status=" << recovery_status_
           << " motion_allowed=true"
           << " map_writes_allowed=" << (map_writes ? "true" : "false")
           << " autonomy_allowed=" << (autonomy ? "true" : "false");
    if (!recovery_reason_.empty()) stream << " reason=" << recovery_reason_;
    return stream.str();
  }

  void publishRecoveryState() {
    std_msgs::msg::String message;
    message.data = recoveryDescription();
    recovery_pub_->publish(message);
    slam_system_interfaces::msg::RecoveryStatus typed;
    if (recovery_status_ == "normal") {
      typed.state = slam_system_interfaces::msg::RecoveryStatus::NORMAL;
    } else if (recovery_status_ == "suspected") {
      typed.state = slam_system_interfaces::msg::RecoveryStatus::SUSPECTED;
    } else if (recovery_status_ == "recovering") {
      typed.state = slam_system_interfaces::msg::RecoveryStatus::RECOVERING;
    } else if (recovery_status_ == "operator_assistance_required") {
      typed.state =
          slam_system_interfaces::msg::RecoveryStatus::OPERATOR_ASSISTANCE_REQUIRED;
    } else {
      typed.state =
          slam_system_interfaces::msg::RecoveryStatus::RECOVERED_WAITING_CONFIRMATION;
    }
    const bool normal = typed.state == slam_system_interfaces::msg::RecoveryStatus::NORMAL;
    typed.motion_allowed = true;
    typed.map_writes_allowed = normal && !recovery_confirmation_required_;
    typed.autonomy_allowed = normal && !recovery_confirmation_required_;
    typed.reason = recovery_reason_;
    recovery_typed_pub_->publish(typed);
  }

  void failActiveMode(const std::string& error) {
    const auto state = manager_.snapshot();
    if (state.mode != SystemMode::kMapping &&
        state.mode != SystemMode::kIncrementalMapping) {
      return;
    }
    const auto exiting = manager_.requestExit();
    if (!exiting.accepted) return;
    RCLCPP_ERROR(get_logger(), "mode startup failed: %s", error.c_str());
    publishState();
    exit_completion_timer_ = create_wall_timer(
        std::chrono::milliseconds(100), [this, error]() {
          exit_completion_timer_->cancel();
          manager_.completeExit(false, error);
          publishState();
        });
  }

  SystemModeManager manager_;
  LocalizationQualityMonitor quality_monitor_;
  std::string map_directory_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<slam_system_interfaces::msg::SystemMode>::SharedPtr state_typed_pub_;
  rclcpp::Subscription<slam_system_interfaces::msg::MappingModeResult>::SharedPtr
      mapping_result_sub_;
  rclcpp::Subscription<slam_system_interfaces::msg::LidarStreamStatus>::SharedPtr
      lidar_recovery_sub_;
  rclcpp::Subscription<slam_system_interfaces::msg::LocalizationHealth>::SharedPtr
      localization_health_sub_;
  rclcpp::Subscription<slam_system_interfaces::msg::RelocalizationStatus>::SharedPtr
      relocalization_recovery_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr recovery_pub_;
  rclcpp::Publisher<slam_system_interfaces::msg::RecoveryStatus>::SharedPtr
      recovery_typed_pub_;
  rclcpp::TimerBase::SharedPtr exit_completion_timer_;
  rclcpp::Service<Trigger>::SharedPtr enter_mapping_;
  rclcpp::Service<Trigger>::SharedPtr enter_incremental_mapping_;
  rclcpp::Service<Trigger>::SharedPtr enter_relocalization_;
  rclcpp::Service<Trigger>::SharedPtr exit_mode_;
  rclcpp::Service<Trigger>::SharedPtr complete_exit_;
  rclcpp::Service<Trigger>::SharedPtr status_;
  rclcpp::Service<Trigger>::SharedPtr recovery_status_service_;
  rclcpp::Service<Trigger>::SharedPtr recovery_ack_service_;
  bool recovery_pending_ = false;
  bool recovery_confirmation_required_ = false;
  std::string recovery_status_ = "normal";
  std::string recovery_reason_;
};

}  // namespace kiss_matcher_slam

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kiss_matcher_slam::SystemModeManagerNode>());
  rclcpp::shutdown();
  return 0;
}
