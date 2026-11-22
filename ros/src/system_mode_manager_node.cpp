#include <chrono>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "kiss_matcher_slam/core/system_mode_manager.hpp"

namespace kiss_matcher_slam {

class SystemModeManagerNode final : public rclcpp::Node {
 public:
  using Trigger = std_srvs::srv::Trigger;

  SystemModeManagerNode() : Node("system_mode_manager") {
    map_directory_ = declare_parameter<std::string>("map_directory", "");
    state_pub_ = create_publisher<std_msgs::msg::String>(
        "system_mode", rclcpp::QoS(1).reliable().transient_local());
    relocalization_state_sub_ = create_subscription<std_msgs::msg::Bool>(
        "relocalization/localized", rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::Bool::ConstSharedPtr message) {
          if (message->data) completeRelocalization();
        });
    mapping_result_sub_ = create_subscription<std_msgs::msg::String>(
        "mapping/mode_result", rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::String::ConstSharedPtr message) {
          if (message->data.rfind("error:", 0) == 0) {
            const auto error = message->data.substr(6);
            if (manager_.snapshot().mode == SystemMode::kExiting) {
              manager_.completeExit(false, error);
              publishState();
            } else {
              failActiveMode(error);
            }
          } else if (message->data == "ready:exited" &&
                     manager_.snapshot().mode == SystemMode::kExiting) {
            manager_.completeExit(true);
            publishState();
          }
        });

    enter_mapping_ = create_service<std_srvs::srv::Trigger>(
        "mode/enter_mapping",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          respond(manager_.requestEnter(SystemMode::kMapping), response);
        });
    enter_incremental_mapping_ = create_service<std_srvs::srv::Trigger>(
        "mode/enter_incremental_mapping",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          respond(manager_.requestEnter(SystemMode::kIncrementalMapping, mapDirectory()), response);
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

    publishState();
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
    std_msgs::msg::String message;
    message.data = describe(manager_.snapshot());
    state_pub_->publish(message);
  }

  void completeRelocalization() {
    if (manager_.snapshot().mode != SystemMode::kRelocalization) return;

    manager_.setLocalizationState(true, true, mapDirectory());
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
  std::string map_directory_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr relocalization_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mapping_result_sub_;
  rclcpp::TimerBase::SharedPtr exit_completion_timer_;
  rclcpp::Service<Trigger>::SharedPtr enter_mapping_;
  rclcpp::Service<Trigger>::SharedPtr enter_incremental_mapping_;
  rclcpp::Service<Trigger>::SharedPtr enter_relocalization_;
  rclcpp::Service<Trigger>::SharedPtr exit_mode_;
  rclcpp::Service<Trigger>::SharedPtr complete_exit_;
  rclcpp::Service<Trigger>::SharedPtr status_;
};

}  // namespace kiss_matcher_slam

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kiss_matcher_slam::SystemModeManagerNode>());
  rclcpp::shutdown();
  return 0;
}
