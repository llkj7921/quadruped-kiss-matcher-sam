#pragma once

#ifndef KISS_MATCHER_POSE_GRAPH_MANAGER_H
#define KISS_MATCHER_POSE_GRAPH_MANAGER_H

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>

// message_filters in ROS2
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/Values.h>

#include "../tictoc.hpp"
#include "kiss_matcher_slam/core/gps_altitude_manager.hpp"
#include "kiss_matcher_slam/core/incremental_map_builder.hpp"
#include "kiss_matcher_slam/core/keyframe_processor.hpp"
#include "kiss_matcher_slam/core/keyframe_selector.hpp"
#include "kiss_matcher_slam/core/loop_candidate_queue.hpp"
#include "kiss_matcher_slam/core/loop_constraint_processor.hpp"
#include "kiss_matcher_slam/core/map_database.hpp"
#include "kiss_matcher_slam/core/mapping_runtime_state.hpp"
#include "kiss_matcher_slam/core/map_product_writer.hpp"
#include "kiss_matcher_slam/core/map_product_loader.hpp"
#include "kiss_matcher_slam/core/pose_graph_backend.hpp"
#include "kiss_matcher_slam/core/realtime_pose_tracker.hpp"
#include "kiss_matcher_slam/core/scan_context_loop_detector.hpp"
#include "kiss_matcher_slam/core/visualization_state.hpp"
#include "slam/loop_closure.h"
#include "slam/loop_registration_coordinator.hpp"
#include "slam/pose_graph_node.hpp"
#include "slam/utils.hpp"

using NodeSyncPolicy =
    message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry,
                                                    sensor_msgs::msg::PointCloud2>;

class PoseGraphManager : public rclcpp::Node {
 public:
  PoseGraphManager() = delete;
  explicit PoseGraphManager(const rclcpp::NodeOptions &options);
  ~PoseGraphManager();

 private:
  void appendKeyframePose(const kiss_matcher::PoseGraphNode &node);

  void callbackNode(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
                    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &scan_msg);
  void modeCallback(const std_msgs::msg::String::ConstSharedPtr &message);
  void recoveryStatusCallback(const std_msgs::msg::String::ConstSharedPtr &message);
  nav_msgs::msg::Odometry odometryInMap(const nav_msgs::msg::Odometry &odom) const;
  void buildMap();
  void detectLoopClosureByScanContext();
  void detectLoopClosureByNNSearch();

  void visualizeCurrentData(const Eigen::Matrix4d &current_odom,
                            const rclcpp::Time &timestamp,
                            const std::string &frame_id);
  void visualizePoseGraph();

  void performRegistration();

  void visualizeLoopClosureClouds();

  visualization_msgs::msg::Marker visualizeLoopMarkers(
      const kiss_matcher_slam::VisualizationSnapshot &snapshot) const;
  visualization_msgs::msg::Marker visualizeLoopDetectionRadius(
      const geometry_msgs::msg::Point &latest_position) const;

  void saveFlagCallback(const std_msgs::msg::String::ConstSharedPtr &msg);
  void gpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  void saveKeyframeIncrementally(const kiss_matcher::PoseGraphNode &node);
  void writeTrajectoryProducts();

  std::string map_frame_;
  std::string base_frame_;
  std::string seq_name_;
  std::string output_directory_;

  kiss_matcher::PoseGraphNode current_frame_;
  kiss_matcher_slam::MappingRuntimeState runtime_state_;
  kiss_matcher_slam::MapDatabase map_database_;
  std::unique_ptr<kiss_matcher_slam::KeyframeProcessor> keyframe_processor_;
  std::unique_ptr<kiss_matcher_slam::MapProductWriter> map_product_writer_;
  kiss_matcher_slam::RealtimePoseTracker realtime_pose_tracker_;
  kiss_matcher_slam::VisualizationState visualization_state_;
  std::unique_ptr<kiss_matcher_slam::LoopConstraintProcessor> loop_constraint_processor_;
  std::unique_ptr<kiss_matcher_slam::LoopRegistrationCoordinator>
      loop_registration_coordinator_;
  std::unique_ptr<kiss_matcher_slam::GpsAltitudeManager> gps_altitude_manager_;
  std::unique_ptr<kiss_matcher_slam::IncrementalMapBuilder> map_builder_;

  kiss_matcher_slam::PoseGraphBackend pose_graph_backend_;

  double scan_voxel_res_;
  double map_voxel_res_;
  double save_voxel_res_;
  double loop_detection_radius_;  // Only for visualization

  kiss_matcher_slam::LoopCandidateQueue loop_candidate_queue_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool store_voxelized_scan_ = false;

  bool save_map_pcd_         = false;
  bool save_in_kitti_format_ = false;
  bool save_keyframes_incrementally_ = false;
  bool overwrite_existing_product_ = false;
  bool mode_control_enabled_ = false;
  std::atomic<bool> mapping_active_{true};
  std::atomic<bool> incremental_mapping_active_{false};
  std::atomic<bool> recovery_allows_map_writes_{true};
  bool incremental_map_loaded_ = false;
  std::string incremental_map_directory_;

  std::shared_ptr<kiss_matcher::LoopClosure> loop_closure_;

  bool scan_context_enabled_ = true;
  bool scan_context_verbose_ = false;
  std::unique_ptr<kiss_matcher_slam::ScanContextLoopDetector> scan_context_detector_;

  // ROS2 interface
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr corrected_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr loop_detection_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr loop_detection_radius_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr realtime_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr optimized_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_src_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_tgt_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_coarse_aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_fine_aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_result_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_save_flag_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_system_mode_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_recovery_status_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;

  // message_filters
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> sub_odom_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub_scan_;
  std::shared_ptr<message_filters::Synchronizer<NodeSyncPolicy>> sub_node_;

  // Timers
  rclcpp::TimerBase::SharedPtr map_timer_;
  rclcpp::TimerBase::SharedPtr scan_context_timer_;
  rclcpp::TimerBase::SharedPtr loop_nnsearch_timer_;
  rclcpp::TimerBase::SharedPtr graph_vis_timer_;
  rclcpp::TimerBase::SharedPtr lc_reg_timer_;
  rclcpp::TimerBase::SharedPtr lc_vis_timer_;
};

#endif  // KISS_MATCHER_POSE_GRAPH_MANAGER_H
