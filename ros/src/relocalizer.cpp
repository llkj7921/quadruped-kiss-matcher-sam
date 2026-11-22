#include <chrono>
#include <cmath>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <slam_system_interfaces/msg/system_mode.hpp>
#include <slam_system_interfaces/msg/relocalization_status.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "kiss_matcher_slam/core/map_product_loader.hpp"
#include "kiss_matcher_slam/core/frame_and_transform_contract.hpp"
#include "kiss_matcher_slam/core/scan_context_loop_detector.hpp"
#include "slam/loop_closure.h"
#include "slam/utils.hpp"

namespace {

Eigen::Matrix4d odometryPose(const nav_msgs::msg::Odometry &msg) {
  Eigen::Quaterniond q(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                       msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
  if (q.norm() < 1e-9) throw std::runtime_error("incoming odometry has a zero quaternion");
  q.normalize();
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose.block<3, 3>(0, 0) = q.toRotationMatrix();
  pose.block<3, 1>(0, 3) = Eigen::Vector3d(msg.pose.pose.position.x,
                                             msg.pose.pose.position.y,
                                             msg.pose.pose.position.z);
  return pose;
}

}  // namespace

class MapRelocalizer final : public rclcpp::Node {
 public:
  MapRelocalizer() : Node("kiss_matcher_relocalizer") {
    const auto map_directory = declare_parameter<std::string>("map_directory", "");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    const auto query_period_s = declare_parameter<double>("query_period_s", 1.0);
    required_confirmations_ = declare_parameter<int>("required_confirmations", 2);
    confirmation_translation_m_ = declare_parameter<double>("confirmation_translation_m", 1.0);
    confirmation_rotation_rad_ =
        declare_parameter<double>("confirmation_rotation_deg", 15.0) * M_PI / 180.0;
    recovery_timeout_s_ = declare_parameter<double>("recovery_timeout_s", 10.0);
    if (map_directory.empty() || !std::isfinite(query_period_s) || query_period_s <= 0.0 ||
        required_confirmations_ <= 0 || !std::isfinite(confirmation_translation_m_) ||
        confirmation_translation_m_ <= 0.0 || !std::isfinite(confirmation_rotation_rad_) ||
        confirmation_rotation_rad_ <= 0.0 || !std::isfinite(recovery_timeout_s_) ||
        recovery_timeout_s_ <= 0.0 ||
        !kiss_matcher_slam::distinctMapAndOdometryFrames(map_frame_, odom_frame_)) {
      throw std::invalid_argument("invalid relocalization configuration");
    }
    query_period_s_ = query_period_s;
    mode_control_enabled_ = declare_parameter<bool>("mode_control_enabled", true);

    kiss_matcher_slam::ScanContextConfig scan_context;
    scan_context.num_rings = declare_parameter<int>("scan_context.num_rings", 20);
    scan_context.num_sectors = declare_parameter<int>("scan_context.num_sectors", 60);
    scan_context.num_candidates = declare_parameter<int>("scan_context.num_candidates", 10);
    scan_context.max_radius_m = declare_parameter<double>("scan_context.max_radius", 80.0);
    scan_context.distance_threshold = declare_parameter<double>("scan_context.distance_threshold", 0.18);
    scan_context.sector_search_ratio = declare_parameter<double>("scan_context.sector_search_ratio", 0.1);
    scan_context.exclude_recent = 0;
    scan_context.minimum_time_separation_s = 0.0;
    detector_ = std::make_unique<kiss_matcher_slam::ScanContextLoopDetector>(scan_context);

    kiss_matcher::LoopClosureConfig registration;
    registration.voxel_res_ = declare_parameter<double>("voxel_resolution", 0.3);
    const int num_submap_keyframes = declare_parameter<int>("num_submap_keyframes", 1);
    registration.enable_global_registration_ = true;
    const int num_inliers_threshold =
        declare_parameter<int>("global_reg.num_inliers_threshold", 50);
    if (num_submap_keyframes <= 0 || num_inliers_threshold <= 0) {
      throw std::invalid_argument(
          "num_submap_keyframes and global_reg.num_inliers_threshold must be positive");
    }
    registration.num_submap_keyframes_ =
        static_cast<std::size_t>(num_submap_keyframes);
    registration.num_inliers_threshold_ =
        static_cast<std::size_t>(num_inliers_threshold);
    registration.matcher_config_ = kiss_matcher::KISSMatcherConfig(registration.voxel_res_, false);
    registration.gicp_config_.num_threads_ = declare_parameter<int>("local_reg.num_threads", 8);
    registration.gicp_config_.correspondence_randomness_ =
        declare_parameter<int>("local_reg.correspondences_number", 20);
    registration.gicp_config_.max_num_iter_ = declare_parameter<int>("local_reg.max_num_iter", 32);
    registration.gicp_config_.scale_factor_for_corr_dist_ =
        declare_parameter<double>("local_reg.scale_factor_for_corr_dist", 5.0);
    registration.gicp_config_.overlap_threshold_ =
        declare_parameter<double>("local_reg.overlap_threshold", 55.0);
    if (!std::isfinite(scan_context.max_radius_m) || scan_context.max_radius_m <= 0.0 ||
        !std::isfinite(scan_context.distance_threshold) ||
        scan_context.distance_threshold <= 0.0 ||
        !std::isfinite(scan_context.sector_search_ratio) ||
        scan_context.sector_search_ratio <= 0.0 ||
        !std::isfinite(registration.voxel_res_) || registration.voxel_res_ <= 0.0 ||
        !std::isfinite(registration.gicp_config_.scale_factor_for_corr_dist_) ||
        registration.gicp_config_.scale_factor_for_corr_dist_ <= 0.0 ||
        !std::isfinite(registration.gicp_config_.overlap_threshold_) ||
        registration.gicp_config_.overlap_threshold_ <= 0.0) {
      throw std::invalid_argument("floating-point relocalization parameters must be finite and positive");
    }
    registration_ = std::make_unique<kiss_matcher::LoopClosure>(registration, get_logger());
    voxel_resolution_ = registration.voxel_res_;
    submap_keyframes_ = registration.num_submap_keyframes_;

    reference_keyframes_ = kiss_matcher_slam::MapProductLoader::load(map_directory);
    RCLCPP_INFO(get_logger(), "Loaded %zu reference keyframes from %s", reference_keyframes_.size(),
                map_directory.c_str());

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    localized_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/relocalization/localized", rclcpp::QoS(1).reliable().transient_local());
    recovery_status_pub_ = create_publisher<std_msgs::msg::String>(
        "/relocalization/recovery_status", rclcpp::QoS(1).reliable().transient_local());
    relocalization_status_pub_ =
        create_publisher<slam_system_interfaces::msg::RelocalizationStatus>(
            "/relocalization/status", rclcpp::QoS(1).reliable().transient_local());
    recovery_watchdog_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
      if (!relocalization_active_ || assistance_published_) return;
      if (now().seconds() - relocalization_start_s_ < recovery_timeout_s_) return;
      assistance_published_ = true;
      std_msgs::msg::String status;
      status.data = "operator_assistance_required";
      recovery_status_pub_->publish(status);
      slam_system_interfaces::msg::RelocalizationStatus typed;
      typed.state =
          slam_system_interfaces::msg::RelocalizationStatus::OPERATOR_ASSISTANCE_REQUIRED;
      typed.reason = "relocalization timed out; return to a known area or viewpoint";
      relocalization_status_pub_->publish(typed);
      RCLCPP_ERROR(get_logger(),
                   "Relocalization timed out; keep manual motion enabled and return to a known area or viewpoint");
    });
    if (mode_control_enabled_) {
      mode_sub_ = create_subscription<slam_system_interfaces::msg::SystemMode>(
          "/system_mode/state", rclcpp::QoS(1).reliable().transient_local(),
          std::bind(&MapRelocalizer::modeCallback, this, std::placeholders::_1));
    } else {
      relocalization_active_ = true;
      normal_mode_active_ = true;
    }
    using Policy = message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry,
                                                                     sensor_msgs::msg::PointCloud2>;
    odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, "odom");
    cloud_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, "cloud");
    sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(Policy(20), *odom_sub_, *cloud_sub_);
    sync_->registerCallback(std::bind(&MapRelocalizer::callback, this, std::placeholders::_1,
                                      std::placeholders::_2));
    odom_tf_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SensorDataQoS(),
        std::bind(&MapRelocalizer::odometryCallback, this, std::placeholders::_1));
  }

 private:
  void modeCallback(const slam_system_interfaces::msg::SystemMode::ConstSharedPtr message) {
    const bool requested =
        message->mode == slam_system_interfaces::msg::SystemMode::RELOCALIZATION;
    normal_mode_active_ = message->mode == slam_system_interfaces::msg::SystemMode::NORMAL;
    incremental_mapping_active_ =
        message->mode == slam_system_interfaces::msg::SystemMode::INCREMENTAL_MAPPING;
    if (requested && !relocalization_active_) {
      localized_ = false;
      have_tentative_ = false;
      confirmations_ = 0;
      last_query_stamp_ = -std::numeric_limits<double>::infinity();
      relocalization_start_s_ = now().seconds();
      assistance_published_ = false;
      slam_system_interfaces::msg::RelocalizationStatus status;
      status.state = slam_system_interfaces::msg::RelocalizationStatus::SEARCHING;
      status.reason = "relocalization mode activated";
      relocalization_status_pub_->publish(status);
      RCLCPP_INFO(get_logger(), "Relocalization mode activated");
    }
    relocalization_active_ = requested;
  }

  pcl::PointCloud<PointType> referenceSubmap(const std::size_t center) const {
    pcl::PointCloud<PointType> result;
    const auto radius = submap_keyframes_ / 2;
    const auto first = center < radius ? 0 : center - radius;
    const auto last = std::min(reference_keyframes_.size(), center + radius + 1);
    for (std::size_t index = first; index < last; ++index) {
      result += kiss_matcher::transformPcd(reference_keyframes_[index].cloud,
                                           reference_keyframes_[index].optimized_pose);
    }
    return *kiss_matcher::voxelize(result, voxel_resolution_);
  }

  void publishMapToOdom(const nav_msgs::msg::Odometry &odom) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = odom.header.stamp;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    const Eigen::Quaterniond q(map_to_odom_.block<3, 3>(0, 0));
    transform.transform.translation.x = map_to_odom_(0, 3);
    transform.transform.translation.y = map_to_odom_(1, 3);
    transform.transform.translation.z = map_to_odom_(2, 3);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(transform);

    const auto map_to_base = map_to_odom_ * odometryPose(odom);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = odom.header.stamp;
    pose.header.frame_id = map_frame_;
    pose.pose.position.x = map_to_base(0, 3);
    pose.pose.position.y = map_to_base(1, 3);
    pose.pose.position.z = map_to_base(2, 3);
    const Eigen::Quaterniond base_q(map_to_base.block<3, 3>(0, 0));
    pose.pose.orientation.x = base_q.x();
    pose.pose.orientation.y = base_q.y();
    pose.pose.orientation.z = base_q.z();
    pose.pose.orientation.w = base_q.w();
    pose_pub_->publish(pose);
  }

  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom) {
    if (!localized_) return;
    if (mode_control_enabled_ && !normal_mode_active_ &&
        !incremental_mapping_active_) return;
    if (odom->header.frame_id != odom_frame_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ignoring odometry for TF: frame_id '%s' does not match configured odom frame '%s'",
          odom->header.frame_id.c_str(), odom_frame_.c_str());
      return;
    }
    try {
      publishMapToOdom(*odom);
    } catch (const std::exception &error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Ignoring invalid odometry for TF: %s", error.what());
    }
  }

  void callback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom,
                const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud) {
    if (!localized_ && !relocalization_active_) return;
    if (!kiss_matcher_slam::relocalizationInputFramesValid(
            odom->header.frame_id, cloud->header.frame_id, odom_frame_)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ignoring relocalization input: odometry frame='%s', cloud frame='%s', expected='%s'",
          odom->header.frame_id.c_str(), cloud->header.frame_id.c_str(), odom_frame_.c_str());
      return;
    }
    const auto odom_pose = odometryPose(*odom);
    if (localized_) {
      return;
    }
    const double stamp = rclcpp::Time(odom->header.stamp).seconds();
    if (stamp - last_query_stamp_ < query_period_s_) return;
    last_query_stamp_ = stamp;

    pcl::PointCloud<PointType> cloud_in_odom;
    pcl::fromROSMsg(*cloud, cloud_in_odom);
    const auto local_cloud = kiss_matcher::transformPcd(cloud_in_odom, odom_pose.inverse());
    const auto candidates = detector_->findCandidates(local_cloud, reference_keyframes_);
    if (candidates.empty()) {
      RCLCPP_INFO(get_logger(), "Relocalization: no Scan Context candidate");
      return;
    }

    for (const auto &candidate : candidates) {
      const auto target = referenceSubmap(candidate.keyframe_id);
      const auto result = registration_->coarseToFineAlignment(local_cloud, target);
      if (!result.is_valid_) continue;
      const Eigen::Matrix4d proposed_map_to_odom = result.pose_ * odom_pose.inverse();
      if (!have_tentative_ || candidate.keyframe_id != tentative_candidate_ ||
          !kiss_matcher_slam::transformsConsistent(
              proposed_map_to_odom, tentative_map_to_odom_,
              confirmation_translation_m_, confirmation_rotation_rad_)) {
        tentative_candidate_ = candidate.keyframe_id;
        tentative_map_to_odom_ = proposed_map_to_odom;
        confirmations_ = 1;
      } else {
        tentative_map_to_odom_ = proposed_map_to_odom;
        ++confirmations_;
      }
      RCLCPP_INFO(get_logger(),
                  "Relocalization match: map keyframe=%zu SC=%.3f inliers=%zu overlap=%.1f%% confirmation=%d/%d",
                  candidate.keyframe_id, candidate.descriptor_distance, result.num_final_inliers_,
                  result.overlapness_, confirmations_, required_confirmations_);
      have_tentative_ = true;
      if (confirmations_ >= required_confirmations_) {
        map_to_odom_ = tentative_map_to_odom_;
        localized_ = true;
        relocalization_active_ = false;
        RCLCPP_INFO(get_logger(), "Relocalization confirmed: publishing %s -> %s", map_frame_.c_str(), odom_frame_.c_str());
        publishMapToOdom(*odom);
        std_msgs::msg::Bool state;
        state.data = true;
        localized_pub_->publish(state);
        slam_system_interfaces::msg::RelocalizationStatus status;
        status.state = slam_system_interfaces::msg::RelocalizationStatus::LOCALIZED;
        status.reason = "geometric verification confirmed";
        relocalization_status_pub_->publish(status);
      }
      return;
    }
    have_tentative_ = false;
    confirmations_ = 0;
    RCLCPP_WARN(get_logger(), "Relocalization: all %zu candidates failed geometric verification", candidates.size());
  }

  std::string map_frame_;
  std::string odom_frame_;
  double query_period_s_ = 1.0;
  double last_query_stamp_ = -std::numeric_limits<double>::infinity();
  int required_confirmations_ = 2;
  int confirmations_ = 0;
  double confirmation_translation_m_ = 1.0;
  double confirmation_rotation_rad_ = 15.0 * M_PI / 180.0;
  double recovery_timeout_s_ = 10.0;
  double relocalization_start_s_ = 0.0;
  bool localized_ = false;
  bool mode_control_enabled_ = true;
  bool relocalization_active_ = false;
  bool normal_mode_active_ = false;
  bool incremental_mapping_active_ = false;
  bool assistance_published_ = false;
  bool have_tentative_ = false;
  std::size_t tentative_candidate_ = 0;
  double voxel_resolution_ = 0.3;
  std::size_t submap_keyframes_ = 1;
  Eigen::Matrix4d tentative_map_to_odom_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d map_to_odom_ = Eigen::Matrix4d::Identity();
  std::vector<kiss_matcher_slam::Keyframe> reference_keyframes_;
  std::unique_ptr<kiss_matcher_slam::ScanContextLoopDetector> detector_;
  std::unique_ptr<kiss_matcher::LoopClosure> registration_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr localized_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr recovery_status_pub_;
  rclcpp::Publisher<slam_system_interfaces::msg::RelocalizationStatus>::SharedPtr
      relocalization_status_pub_;
  rclcpp::Subscription<slam_system_interfaces::msg::SystemMode>::SharedPtr mode_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_tf_sub_;
  rclcpp::TimerBase::SharedPtr recovery_watchdog_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_sub_;
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>>> sync_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapRelocalizer>());
  rclcpp::shutdown();
  return 0;
}
