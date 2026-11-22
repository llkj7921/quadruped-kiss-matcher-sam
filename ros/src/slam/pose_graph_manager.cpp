#include "slam/pose_graph_manager.h"

#include <filesystem>

#include <Eigen/Geometry>
#include <tf2/exceptions.h>

#include "kiss_matcher_slam/core/frame_and_transform_contract.hpp"

using namespace kiss_matcher;

PoseGraphManager::PoseGraphManager(const rclcpp::NodeOptions &options)
    : rclcpp::Node("km_sam", options) {
  auto config = loadConfig();
  map_frame_ = config.map_frame;
  odom_frame_ = config.odom_frame;
  base_frame_ = config.base_frame;
  mode_control_enabled_ = config.mode_control_enabled;
  incremental_map_directory_ = config.incremental_map_directory;
  mapping_active_ = !mode_control_enabled_;
  store_voxelized_scan_ = config.store_voxelized_scan;
  scan_voxel_res_ = config.loop_closure.voxel_res_;
  map_voxel_res_ = config.map_voxel_resolution;
  save_voxel_res_ = config.save_voxel_resolution;
  scan_context_verbose_ = config.scan_context_verbose;
  scan_context_enabled_ = config.scan_context_enabled;
  save_map_pcd_ = config.save_map_pcd;
  save_in_kitti_format_ = config.save_in_kitti_format;
  seq_name_ = config.sequence_name;
  output_directory_ = config.output_directory;
  save_keyframes_incrementally_ = config.save_keyframes_incrementally;
  overwrite_existing_product_ = config.overwrite_existing_product;
  const double keyframe_translation_threshold_m =
      config.keyframe_processor.selection.translation_threshold_m;
  const double keyframe_rotation_threshold_rad =
      config.keyframe_processor.selection.rotation_threshold_rad;

  loop_constraint_processor_ =
      std::make_unique<kiss_matcher_slam::LoopConstraintProcessor>(
          std::move(config.loop_constraint),
          pose_graph_backend_,
          visualization_state_,
          runtime_state_);
  gps_altitude_manager_ =
      std::make_unique<kiss_matcher_slam::GpsAltitudeManager>(config.gps_altitude);
  keyframe_processor_ = std::make_unique<kiss_matcher_slam::KeyframeProcessor>(
      std::move(config.keyframe_processor),
      map_database_,
      pose_graph_backend_,
      *gps_altitude_manager_,
      runtime_state_);

  rclcpp::QoS qos(1);
  qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  loop_closure_ = std::make_shared<LoopClosure>(config.loop_closure, get_logger());
  loop_detection_radius_ = config.loop_closure.loop_detection_radius_;
  loop_registration_coordinator_ =
      std::make_unique<kiss_matcher_slam::LoopRegistrationCoordinator>(
          loop_candidate_queue_,
          map_database_,
          *loop_closure_,
          *loop_constraint_processor_);

  scan_context_detector_ =
      std::make_unique<kiss_matcher_slam::ScanContextLoopDetector>(config.scan_context);

  initializeRosInterfaces(qos);

  kiss_matcher_slam::MapProductConfig product_config;
  product_config.output_directory = output_directory_;
  product_config.sequence = seq_name_;
  product_config.map_frame = map_frame_;
  product_config.base_frame = base_frame_;
  product_config.save_voxel_resolution = save_voxel_res_;
  product_config.keyframe_translation_threshold_m =
      keyframe_translation_threshold_m;
  product_config.keyframe_rotation_threshold_rad =
      keyframe_rotation_threshold_rad;
  product_config.gps_altitude_enabled = gps_altitude_manager_->enabled();
  product_config.save_keyframes_incrementally = save_keyframes_incrementally_;
  product_config.overwrite_existing = overwrite_existing_product_;
  map_product_writer_ =
      std::make_unique<kiss_matcher_slam::MapProductWriter>(std::move(product_config));
  map_product_writer_->initialize();

  kiss_matcher_slam::IncrementalMapBuilderConfig map_builder_config;
  map_builder_config.scan_voxel_resolution = scan_voxel_res_;
  map_builder_config.map_voxel_resolution = map_voxel_res_;
  map_builder_config.keyframe_clouds_are_voxelized = store_voxelized_scan_;
  map_builder_ =
      std::make_unique<kiss_matcher_slam::IncrementalMapBuilder>(map_builder_config);
  initializeTimers(config.map_update_hz, config.loop_detector_hz,
                   config.enable_loop_nnsearch, config.loop_nnsearch_hz,
                   config.visualization_hz);

  if (!config.loop_closure.is_multilayer_env_) {
    RCLCPP_WARN(
        get_logger(),
        "'loop.is_multilayer_env' is set to `false`. "
        "This setting is recommended for outdoor environments to ignore the effect of Z-drift. "
        "However, if you're running SLAM in an indoor multi-layer environment, "
        "consider setting it to true to enable full 3D NN search for loop candidates.");
  }
  RCLCPP_INFO(this->get_logger(), "Main class, starting node...");
}

PoseGraphManagerConfig PoseGraphManager::loadConfig() {
  PoseGraphManagerConfig config;
  config.map_frame = declare_parameter<std::string>("map_frame", "map");
  config.odom_frame = declare_parameter<std::string>("odom_frame", "odom");
  config.base_frame = declare_parameter<std::string>("base_frame", "base");
  if (!kiss_matcher_slam::distinctMapAndOdometryFrames(config.map_frame,
                                                        config.odom_frame)) {
    throw std::invalid_argument("map_frame and odom_frame must be distinct and non-empty");
  }
  config.loop_detector_hz = declare_parameter<double>("loop_detector_hz", 1.0);
  config.loop_nnsearch_hz = declare_parameter<double>("loop_nnsearch_hz", 1.0);
  config.enable_loop_nnsearch = declare_parameter<bool>("loop.enable_nnsearch", false);
  config.map_update_hz = declare_parameter<double>("map_update_hz", 0.2);
  config.visualization_hz = declare_parameter<double>("vis_hz", 0.5);
  config.mode_control_enabled = declare_parameter<bool>("mode_control_enabled", false);
  config.incremental_map_directory =
      declare_parameter<std::string>("incremental_map_directory", "");
  config.store_voxelized_scan = declare_parameter<bool>("store_voxelized_scan", false);

  auto &loop = config.loop_closure;
  loop.voxel_res_ = declare_parameter<double>("voxel_resolution", 0.3);
  loop.matcher_config_ = KISSMatcherConfig(loop.voxel_res_, false);
  config.map_voxel_resolution = declare_parameter<double>("map_voxel_resolution", 1.0);
  config.save_voxel_resolution = declare_parameter<double>("save_voxel_resolution", 0.3);
  auto &selection = config.keyframe_processor.selection;
  selection.translation_threshold_m =
      declare_parameter<double>("keyframe.keyframe_threshold", 1.0);
  selection.rotation_threshold_rad =
      declare_parameter<double>("keyframe.rotation_threshold_deg", 10.0) * M_PI / 180.0;
  selection.minimum_time_interval_s =
      declare_parameter<double>("keyframe.minimum_time_interval", 0.0);
  loop.num_submap_keyframes_ = declare_parameter<int>("keyframe.num_submap_keyframes", 5);
  loop.verbose_ = declare_parameter<bool>("loop.verbose", false);
  loop.is_multilayer_env_ = declare_parameter<bool>("loop.is_multilayer_env", false);
  loop.loop_detection_radius_ =
      declare_parameter<double>("loop.loop_detection_radius", 15.0);
  loop.loop_detection_timediff_threshold_ =
      declare_parameter<double>("loop.loop_detection_timediff_threshold", 10.0);
  auto &gicp = loop.gicp_config_;
  gicp.num_threads_ = declare_parameter<int>("local_reg.num_threads", 8);
  gicp.correspondence_randomness_ =
      declare_parameter<int>("local_reg.correspondences_number", 20);
  gicp.max_num_iter_ = declare_parameter<int>("local_reg.max_num_iter", 32);
  gicp.scale_factor_for_corr_dist_ =
      declare_parameter<double>("local_reg.scale_factor_for_corr_dist", 5.0);
  gicp.overlap_threshold_ = declare_parameter<double>("local_reg.overlap_threshold", 90.0);
  loop.enable_global_registration_ = declare_parameter<bool>("global_reg.enable", false);
  loop.num_inliers_threshold_ =
      declare_parameter<int>("global_reg.num_inliers_threshold", 100);

  config.scan_context_verbose = declare_parameter<bool>("scan_context.verbose", false);
  config.scan_context_enabled = declare_parameter<bool>("scan_context.enable", true);
  auto &scan_context = config.scan_context;
  scan_context.num_rings = declare_parameter<int>("scan_context.num_rings", 20);
  scan_context.num_sectors = declare_parameter<int>("scan_context.num_sectors", 60);
  scan_context.exclude_recent = declare_parameter<int>("scan_context.exclude_recent", 30);
  scan_context.num_candidates = declare_parameter<int>("scan_context.num_candidates", 10);
  scan_context.max_radius_m = declare_parameter<double>("scan_context.max_radius", 80.0);
  scan_context.distance_threshold =
      declare_parameter<double>("scan_context.distance_threshold", 0.18);
  scan_context.sector_search_ratio =
      declare_parameter<double>("scan_context.sector_search_ratio", 0.1);
  scan_context.minimum_time_separation_s =
      declare_parameter<double>("scan_context.min_time_separation", 30.0);

  config.keyframe_processor.prior_variances = declare_parameter<std::vector<double>>(
      "pose_graph.prior_variances", {1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2});
  config.keyframe_processor.odometry_variances = declare_parameter<std::vector<double>>(
      "pose_graph.odom_variances", {1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2});
  config.loop_constraint.base_variances = declare_parameter<std::vector<double>>(
      "pose_graph.loop_variances", {1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2});
  config.loop_constraint.huber_k =
      declare_parameter<double>("pose_graph.loop_huber_k", 1.345);
  config.loop_constraint.adaptive_noise =
      declare_parameter<bool>("pose_graph.adaptive_loop_noise", true);

  auto &gps = config.gps_altitude;
  gps.enabled = declare_parameter<bool>("gps_altitude.enable", false);
  gps.time_tolerance_s = declare_parameter<double>("gps_altitude.time_tolerance", 0.2);
  gps.xy_variance = declare_parameter<double>("gps_altitude.xy_variance", 1e8);
  gps.z_variance = declare_parameter<double>("gps_altitude.z_variance", 4.0);
  gps.huber_k = declare_parameter<double>("gps_altitude.huber_k", 1.345);

  config.save_map_pcd = declare_parameter<bool>("result.save_map_pcd", false);
  config.save_in_kitti_format =
      declare_parameter<bool>("result.save_in_kitti_format", false);
  config.sequence_name = declare_parameter<std::string>("result.seq_name", "sequence");
  config.output_directory =
      declare_parameter<std::string>("result.output_directory", "maps");
  config.save_keyframes_incrementally =
      declare_parameter<bool>("result.save_keyframes_incrementally", false);
  config.overwrite_existing_product =
      declare_parameter<bool>("result.overwrite_existing", false);
  if (!std::isfinite(config.loop_detector_hz) || config.loop_detector_hz <= 0.0 ||
      !std::isfinite(config.loop_nnsearch_hz) || config.loop_nnsearch_hz <= 0.0 ||
      !std::isfinite(config.map_update_hz) || config.map_update_hz <= 0.0 ||
      !std::isfinite(config.visualization_hz) || config.visualization_hz <= 0.0) {
    throw std::invalid_argument("pose graph timer frequencies must be finite and positive");
  }
  return config;
}

void PoseGraphManager::initializeRosInterfaces(const rclcpp::QoS &latched_qos) {
  // Relative data topics remain namespace-friendly. System-wide control topics
  // intentionally use absolute names until the compatibility migration ends.
  path_pub_ = create_publisher<nav_msgs::msg::Path>("path/original", latched_qos);
  corrected_path_pub_ =
      create_publisher<nav_msgs::msg::Path>("path/corrected", latched_qos);
  map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("global_map", latched_qos);
  scan_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("curr_scan", latched_qos);
  loop_detection_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("loop_detection", latched_qos);
  loop_detection_radius_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "loop_detection_radius", latched_qos);
  realtime_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("pose_stamped", latched_qos);
  optimized_odom_pub_ =
      create_publisher<nav_msgs::msg::Odometry>("odometry/optimized", latched_qos);
  debug_src_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("lc/src", latched_qos);
  debug_tgt_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("lc/tgt", latched_qos);
  debug_coarse_aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "lc/coarse_alignment", latched_qos);
  debug_fine_aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "lc/fine_alignment", latched_qos);
  debug_cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("lc/debug_cloud", latched_qos);

  const auto control_qos = rclcpp::QoS(1).reliable().transient_local();
  mode_result_pub_ =
      create_publisher<std_msgs::msg::String>("/mapping/mode_result", control_qos);
  mode_result_typed_pub_ = create_publisher<slam_system_interfaces::msg::MappingModeResult>(
      "/mapping/mode_state", control_qos);

  sub_odom_ =
      std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, "/odom");
  sub_scan_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
      this, "/cloud");
  sub_node_ = std::make_shared<message_filters::Synchronizer<NodeSyncPolicy>>(
      NodeSyncPolicy(10), *sub_odom_, *sub_scan_);
  sub_node_->registerCallback(std::bind(
      &PoseGraphManager::callbackNode, this, std::placeholders::_1, std::placeholders::_2));

  sub_save_flag_ = create_subscription<std_msgs::msg::String>(
      "save_dir", 1,
      std::bind(&PoseGraphManager::saveFlagCallback, this, std::placeholders::_1));
  sub_recovery_status_ = create_subscription<slam_system_interfaces::msg::RecoveryStatus>(
      "/recovery/state", control_qos,
      std::bind(&PoseGraphManager::recoveryStatusCallback, this, std::placeholders::_1));
  if (mode_control_enabled_) {
    sub_system_mode_ = create_subscription<slam_system_interfaces::msg::SystemMode>(
        "/system_mode/state", control_qos,
        std::bind(&PoseGraphManager::modeCallback, this, std::placeholders::_1));
  }
  if (gps_altitude_manager_->enabled()) {
    sub_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "gps/fix", rclcpp::SensorDataQoS(),
        std::bind(&PoseGraphManager::gpsCallback, this, std::placeholders::_1));
  }
}

void PoseGraphManager::initializeTimers(const double map_update_hz,
                                        const double loop_detector_hz,
                                        const bool enable_loop_nnsearch,
                                        const double loop_nnsearch_hz,
                                        const double visualization_hz) {
  map_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / map_update_hz),
                                 std::bind(&PoseGraphManager::buildMap, this));
  scan_context_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / loop_detector_hz),
      std::bind(&PoseGraphManager::detectLoopClosureByScanContext, this));
  if (enable_loop_nnsearch) {
    loop_nnsearch_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / loop_nnsearch_hz),
        std::bind(&PoseGraphManager::detectLoopClosureByNNSearch, this));
  }
  graph_vis_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / visualization_hz),
      std::bind(&PoseGraphManager::visualizePoseGraph, this));
  lc_reg_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / 100.0),
                                    std::bind(&PoseGraphManager::performRegistration, this));
  // 20 Hz is enough as long as it is faster than the full registration process.
  lc_vis_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / 20.0),
      std::bind(&PoseGraphManager::visualizeLoopClosureClouds, this));
}

PoseGraphManager::~PoseGraphManager() {
  const auto keyframes = map_database_.snapshot();
  if (save_map_pcd_ && !keyframes.empty()) {
    try {
      map_product_writer_->writeMap(keyframes, true);
      writeTrajectoryProducts();
      RCLCPP_INFO(this->get_logger(),
                  "Mapping data product saved to %s",
                  map_product_writer_->sequenceDirectory().c_str());
    } catch (const std::exception &error) {
      RCLCPP_ERROR(this->get_logger(), "Failed to save mapping data product: %s", error.what());
    }
  }
}

void PoseGraphManager::gpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
  if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX ||
      !std::isfinite(msg->altitude)) {
    return;
  }
  gps_altitude_manager_->update(rclcpp::Time(msg->header.stamp).seconds(), msg->altitude);
}

void PoseGraphManager::saveKeyframeIncrementally(const PoseGraphNode &node) {
  map_product_writer_->appendKeyframe(node);
}

void PoseGraphManager::writeTrajectoryProducts() {
  const auto estimates = pose_graph_backend_.estimates();
  if (estimates.empty()) return;
  const auto keyframes = map_database_.snapshot();
  std::vector<Eigen::Matrix4d> optimized_poses;
  optimized_poses.reserve(estimates.size());
  for (size_t i = 0; i < estimates.size(); ++i) {
    optimized_poses.push_back(gtsamToEigen(estimates.at<gtsam::Pose3>(i)));
  }
  map_product_writer_->writeOptimizedTrajectory(keyframes, optimized_poses);
}

void PoseGraphManager::appendKeyframePose(const PoseGraphNode &node) {
  visualization_state_.appendKeyframe(node.pose_, node.pose_corrected_);
}

void PoseGraphManager::callbackNode(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
                                    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &scan_msg) {
  if (!mapping_active_ || !recovery_allows_map_writes_) return;
  if (odom_msg->header.frame_id != odom_frame_ || scan_msg->header.frame_id != odom_frame_) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting mapping input with invalid frame contract: odom=%s cloud=%s expected=%s",
        odom_msg->header.frame_id.c_str(), scan_msg->header.frame_id.c_str(),
        odom_frame_.c_str());
    return;
  }
  const size_t latest_keyframe_idx = keyframe_processor_->nextKeyframeId();

  // NOTE(hlim): For clarification, 'current' refers to the real-time incoming messages,
  // while 'latest' indicates the last keyframe stored in map_database_.
  Eigen::Matrix4d current_odom = current_frame_.pose_;
  current_frame_ = PoseGraphNode(
      *odom_msg, *scan_msg, latest_keyframe_idx, scan_voxel_res_, store_voxelized_scan_);
  if (incremental_mapping_active_) {
    try {
      const auto aligned_odom = odometryInMap(*odom_msg);
      const Eigen::Quaterniond aligned_q(
          aligned_odom.pose.pose.orientation.w, aligned_odom.pose.pose.orientation.x,
          aligned_odom.pose.pose.orientation.y, aligned_odom.pose.pose.orientation.z);
      current_frame_.pose_.block<3, 3>(0, 0) = aligned_q.normalized().toRotationMatrix();
      current_frame_.pose_.block<3, 1>(0, 3) = Eigen::Vector3d(
          aligned_odom.pose.pose.position.x, aligned_odom.pose.pose.position.y,
          aligned_odom.pose.pose.position.z);
      current_frame_.pose_corrected_ = current_frame_.pose_;
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Waiting for map-to-odometry transform: %s", error.what());
      return;
    }
  }

  kiss_matcher::TicToc total_timer;

  visualizeCurrentData(current_odom, odom_msg->header.stamp, scan_msg->header.frame_id);

  const auto keyframe_result = keyframe_processor_->process(current_frame_);
  if (keyframe_result.initialized_session) {
    appendKeyframePose(current_frame_);
    saveKeyframeIncrementally(current_frame_);
    RCLCPP_INFO(this->get_logger(), "The first node comes. Initialization complete.");
    return;
  }
  if (!keyframe_result.accepted) return;

  saveKeyframeIncrementally(current_frame_);
  appendKeyframePose(current_frame_);
  realtime_pose_tracker_.applyCorrection(keyframe_result.latest_optimized_pose);
  writeTrajectoryProducts();

  const auto t_total = total_timer.toc();
  const auto t_keyframe_processing =
      std::max(0.0, t_total - keyframe_result.optimization_time_ms);
  RCLCPP_INFO(
      this->get_logger(),
      "# of Keyframes: %zu. Timing (msec) → Total: %.1f | Keyframe: %.1f | Optim.: %.1f",
      map_database_.size(),
      t_total,
      t_keyframe_processing,
      keyframe_result.optimization_time_ms);
}

void PoseGraphManager::recoveryStatusCallback(
    const slam_system_interfaces::msg::RecoveryStatus::ConstSharedPtr &message) {
  const bool allowed = message->map_writes_allowed;
  const bool previously_allowed = recovery_allows_map_writes_.exchange(allowed);
  if (previously_allowed && !allowed) {
    RCLCPP_WARN(get_logger(), "Map writes paused by localization health: %s",
                message->reason.c_str());
  } else if (!previously_allowed && allowed) {
    RCLCPP_INFO(get_logger(), "Map writes resumed after localization recovery");
  }
}

void PoseGraphManager::modeCallback(
    const slam_system_interfaces::msg::SystemMode::ConstSharedPtr &message) {
  const bool mapping = message->mode == slam_system_interfaces::msg::SystemMode::MAPPING;
  const bool incremental =
      message->mode == slam_system_interfaces::msg::SystemMode::INCREMENTAL_MAPPING;
  const bool exiting = message->mode == slam_system_interfaces::msg::SystemMode::EXITING;
  const bool was_mapping_active = mapping_active_.exchange(false);
  incremental_mapping_active_ = false;

  if (exiting && was_mapping_active) {
    try {
      writeTrajectoryProducts();
      if (save_map_pcd_ && !map_database_.empty()) {
        map_product_writer_->writeMap(map_database_.snapshot(), true);
      }
      publishModeResult(slam_system_interfaces::msg::MappingModeResult::READY_EXITED);
    } catch (const std::exception &error) {
      publishModeResult(slam_system_interfaces::msg::MappingModeResult::ERROR, error.what());
    }
    return;
  }

  if (mapping) {
    if (!map_database_.empty()) {
      RCLCPP_ERROR(get_logger(), "A mapping session already exists; restart the node for a new map");
      publishModeResult(
          slam_system_interfaces::msg::MappingModeResult::ERROR,
          "a mapping session already exists; restart the node for a new map");
      return;
    }
    mapping_active_ = true;
    publishModeResult(slam_system_interfaces::msg::MappingModeResult::READY_MAPPING);
    return;
  }
  if (!incremental) return;

  try {
    if (!incremental_map_loaded_) {
      if (incremental_map_directory_.empty()) {
        throw std::runtime_error("incremental_map_directory is empty");
      }
      const auto keyframes =
          kiss_matcher_slam::MapProductLoader::load(incremental_map_directory_);
      keyframe_processor_->restore(keyframes);
      for (const auto &frame : keyframes) {
        appendKeyframePose(PoseGraphNode(frame));
        map_product_writer_->appendKeyframe(frame);
      }
      writeTrajectoryProducts();
      incremental_map_loaded_ = true;
      RCLCPP_INFO(get_logger(), "Restored %zu keyframes for incremental mapping",
                  keyframes.size());
    }
    incremental_mapping_active_ = true;
    mapping_active_ = true;
    publishModeResult(
        slam_system_interfaces::msg::MappingModeResult::READY_INCREMENTAL_MAPPING);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(), "Cannot enter incremental mapping: %s", error.what());
    publishModeResult(slam_system_interfaces::msg::MappingModeResult::ERROR, error.what());
  }
}

void PoseGraphManager::publishModeResult(const uint8_t state, const std::string &error) {
  slam_system_interfaces::msg::MappingModeResult typed;
  typed.state = state;
  typed.error = error;
  mode_result_typed_pub_->publish(typed);

  std_msgs::msg::String legacy;
  if (state == slam_system_interfaces::msg::MappingModeResult::READY_MAPPING) {
    legacy.data = "ready:mapping";
  } else if (state ==
             slam_system_interfaces::msg::MappingModeResult::READY_INCREMENTAL_MAPPING) {
    legacy.data = "ready:incremental_mapping";
  } else if (state == slam_system_interfaces::msg::MappingModeResult::READY_EXITED) {
    legacy.data = "ready:exited";
  } else {
    legacy.data = "error:" + error;
  }
  mode_result_pub_->publish(legacy);
}

nav_msgs::msg::Odometry PoseGraphManager::odometryInMap(
    const nav_msgs::msg::Odometry &odom) const {
  const auto transform = tf_buffer_->lookupTransform(
      map_frame_, odom.header.frame_id, odom.header.stamp, rclcpp::Duration::from_seconds(0.05));
  Eigen::Quaterniond q(transform.transform.rotation.w, transform.transform.rotation.x,
                       transform.transform.rotation.y, transform.transform.rotation.z);
  Eigen::Matrix4d map_to_odom = Eigen::Matrix4d::Identity();
  map_to_odom.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  map_to_odom.block<3, 1>(0, 3) =
      Eigen::Vector3d(transform.transform.translation.x,
                      transform.transform.translation.y,
                      transform.transform.translation.z);
  Eigen::Quaterniond odom_q(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                            odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
  Eigen::Matrix4d odom_pose = Eigen::Matrix4d::Identity();
  odom_pose.block<3, 3>(0, 0) = odom_q.normalized().toRotationMatrix();
  odom_pose.block<3, 1>(0, 3) = Eigen::Vector3d(
      odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
  const Eigen::Matrix4d pose = map_to_odom * odom_pose;

  auto result = odom;
  result.header.frame_id = map_frame_;
  result.pose.pose.position.x = pose(0, 3);
  result.pose.pose.position.y = pose(1, 3);
  result.pose.pose.position.z = pose(2, 3);
  const Eigen::Quaterniond result_q(pose.block<3, 3>(0, 0));
  result.pose.pose.orientation.x = result_q.x();
  result.pose.pose.orientation.y = result_q.y();
  result.pose.pose.orientation.z = result_q.z();
  result.pose.pose.orientation.w = result_q.w();
  return result;
}

void PoseGraphManager::buildMap() {
  if (map_pub_->get_subscription_count() > 0) {
    const bool rebuild_map = runtime_state_.consumeMapRebuildRequest();
    auto keyframes = map_database_.snapshot();
    if (keyframes.empty()) return;
    auto map_result = map_builder_->build(keyframes, rebuild_map);
    for (auto &[id, cloud] : map_result.visualization_cache_updates) {
      if (!map_database_.setVisualizationCloud(id, std::move(cloud))) {
        throw std::runtime_error("failed to cache a keyframe visualization cloud");
      }
    }
    map_pub_->publish(
        toROSMsg(map_result.cloud, map_frame_, toRclcppTime(keyframes.back().timestamp)));
  }
}

void PoseGraphManager::detectLoopClosureByScanContext() {
  if (!scan_context_enabled_ || !runtime_state_.initialized() || map_database_.empty()) return;
  const auto claimed_id = map_database_.claimLatestForPlaceRecognition();
  if (!claimed_id) return;

  auto stored_keyframes = map_database_.snapshot();
  stored_keyframes.resize(*claimed_id + 1);
  const auto &query = stored_keyframes.back();

  const auto candidate = scan_context_detector_->detect(query, stored_keyframes);
  if (candidate) {
    loop_candidate_queue_.push({query.id, candidate->keyframe_id});
    if (scan_context_verbose_) {
      RCLCPP_INFO(get_logger(),
                  "Scan Context candidate: query=%zu match=%zu distance=%.3f yaw=%.3f rad",
                  query.id,
                  candidate->keyframe_id,
                  candidate->descriptor_distance,
                  candidate->yaw_offset_rad);
    }
  }
}

void PoseGraphManager::detectLoopClosureByNNSearch() {
  if (!runtime_state_.initialized() || map_database_.empty()) return;
  const auto claimed_id = map_database_.claimLatestForProximitySearch();
  if (!claimed_id) return;

  auto stored_keyframes = map_database_.snapshot();
  stored_keyframes.resize(*claimed_id + 1);
  const auto &query = stored_keyframes.back();

  const auto &loop_idx_pairs = loop_closure_->fetchLoopCandidates(query, stored_keyframes);

  for (const auto &loop_candidate : loop_idx_pairs) {
    loop_candidate_queue_.push(loop_candidate);
  }
}

void PoseGraphManager::performRegistration() {
  const auto outcome = loop_registration_coordinator_->processNext();
  using Status = kiss_matcher_slam::LoopRegistrationStatus;
  if (outcome.status == Status::kNoWork) return;
  if (outcome.status == Status::kInvalidCandidate) {
    RCLCPP_ERROR(get_logger(), "Discarding loop candidate with an invalid keyframe index");
    return;
  }
  if (outcome.status == Status::kAccepted) {
    RCLCPP_INFO(this->get_logger(), "LC accepted. Overlapness: %.3f", outcome.overlap_percent);
  } else if (outcome.status == Status::kRejectedMatcher) {
    RCLCPP_WARN(this->get_logger(), "LC rejected. KISS-Matcher failed");
  } else if (outcome.status == Status::kRejectedOverlap) {
    RCLCPP_WARN(this->get_logger(), "LC rejected. Overlapness: %.3f", outcome.overlap_percent);
  }
  RCLCPP_INFO(this->get_logger(), "Reg: %.1f msec", outcome.elapsed_ms);
}

void PoseGraphManager::visualizeCurrentData(const Eigen::Matrix4d &current_odom,
                                            const rclcpp::Time &timestamp,
                                            const std::string &frame_id) {
  // NOTE(hlim): Instead of visualizing only when adding keyframes (node-wise), which can feel
  // choppy, we visualize the current frame every cycle to ensure smoother, real-time visualization.
  current_frame_.pose_corrected_ =
      realtime_pose_tracker_.advance(current_odom, current_frame_.pose_);

  geometry_msgs::msg::PoseStamped ps =
      eigenToPoseStamped(current_frame_.pose_corrected_, map_frame_);
  realtime_pose_pub_->publish(ps);

  nav_msgs::msg::Odometry optimized_odom;
  optimized_odom.header = ps.header;
  optimized_odom.child_frame_id = base_frame_.empty() ? frame_id : base_frame_;
  optimized_odom.pose.pose = ps.pose;
  optimized_odom_pub_->publish(optimized_odom);

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp    = timestamp;
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id  = base_frame_.empty() ? frame_id : base_frame_;
  Eigen::Quaterniond q(current_frame_.pose_corrected_.block<3, 3>(0, 0));
  transform_stamped.transform.translation.x = current_frame_.pose_corrected_(0, 3);
  transform_stamped.transform.translation.y = current_frame_.pose_corrected_(1, 3);
  transform_stamped.transform.translation.z = current_frame_.pose_corrected_(2, 3);
  transform_stamped.transform.rotation.x    = q.x();
  transform_stamped.transform.rotation.y    = q.y();
  transform_stamped.transform.rotation.z    = q.z();
  transform_stamped.transform.rotation.w    = q.w();
  tf_broadcaster_->sendTransform(transform_stamped);

  scan_pub_->publish(toROSMsg(
      transformPcd(current_frame_.scan_, current_frame_.pose_corrected_), map_frame_, timestamp));
  const auto latest_corrected_pose = visualization_state_.latestCorrectedPose();
  if (latest_corrected_pose) {
    geometry_msgs::msg::Point latest_position;
    latest_position.x = (*latest_corrected_pose)(0, 3);
    latest_position.y = (*latest_corrected_pose)(1, 3);
    latest_position.z = (*latest_corrected_pose)(2, 3);
    loop_detection_radius_pub_->publish(
        visualizeLoopDetectionRadius(latest_position));
  }
}

void PoseGraphManager::visualizePoseGraph() {
  if (!runtime_state_.initialized()) {
    return;
  }

  if (runtime_state_.consumeGraphRefreshRequest()) {
    const auto estimates = pose_graph_backend_.estimates();
    std::vector<Eigen::Matrix4d> corrected_poses;
    corrected_poses.reserve(estimates.size());
    for (size_t i = 0; i < estimates.size(); ++i) {
      corrected_poses.push_back(gtsamToEigen(estimates.at<gtsam::Pose3>(i)));
    }
    visualization_state_.updateCorrectedPoses(std::move(corrected_poses));
  }

  const auto snapshot = visualization_state_.snapshot();
  nav_msgs::msg::Path odometry_path;
  nav_msgs::msg::Path corrected_path;
  odometry_path.header.frame_id = map_frame_;
  corrected_path.header.frame_id = map_frame_;
  for (const auto &pose : snapshot.odometry_poses) {
    odometry_path.poses.push_back(eigenToPoseStamped(pose, map_frame_));
  }
  for (const auto &pose : snapshot.corrected_poses) {
    corrected_path.poses.push_back(eigenToPoseStamped(pose, map_frame_));
  }
  path_pub_->publish(odometry_path);
  corrected_path_pub_->publish(corrected_path);
  if (!snapshot.loop_edges.empty()) loop_detection_pub_->publish(visualizeLoopMarkers(snapshot));
}

void PoseGraphManager::visualizeLoopClosureClouds() {
  const auto debug = loop_registration_coordinator_->consumeDebugClouds();
  if (!debug) return;
  const auto keyframes = map_database_.snapshot();
  if (debug->query_id >= keyframes.size()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Invalid successful loop-closure keyframe index: %zu (keyframe count: %zu)",
                 debug->query_id,
                 keyframes.size());
    return;
  }
  const auto query_timestamp = toRclcppTime(keyframes[debug->query_id].timestamp);

  debug_src_pub_->publish(toROSMsg(debug->source, map_frame_, query_timestamp));
  debug_tgt_pub_->publish(toROSMsg(debug->target, map_frame_, query_timestamp));
  debug_fine_aligned_pub_->publish(
      toROSMsg(debug->final_aligned, map_frame_, query_timestamp));
  debug_coarse_aligned_pub_->publish(
      toROSMsg(debug->coarse_aligned, map_frame_, query_timestamp));
  debug_cloud_pub_->publish(toROSMsg(debug->debug, map_frame_, query_timestamp));
}

visualization_msgs::msg::Marker PoseGraphManager::visualizeLoopMarkers(
    const kiss_matcher_slam::VisualizationSnapshot &snapshot) const {
  visualization_msgs::msg::Marker edges;
  edges.type               = visualization_msgs::msg::Marker::LINE_LIST;
  edges.scale.x            = 0.12f;
  edges.header.frame_id    = map_frame_;
  edges.pose.orientation.w = 1.0f;
  edges.color.r            = 1.0f;
  edges.color.g            = 1.0f;
  edges.color.b            = 1.0f;
  edges.color.a            = 1.0f;

  for (const auto &[query_id, match_id] : snapshot.loop_edges) {
    if (query_id >= snapshot.corrected_poses.size() ||
        match_id >= snapshot.corrected_poses.size()) {
      continue;
    }
    const auto &pose = snapshot.corrected_poses[query_id];
    const auto &pose2 = snapshot.corrected_poses[match_id];

    geometry_msgs::msg::Point p, p2;
    p.x  = pose(0, 3);
    p.y  = pose(1, 3);
    p.z  = pose(2, 3);
    p2.x = pose2(0, 3);
    p2.y = pose2(1, 3);
    p2.z = pose2(2, 3);

    edges.points.push_back(p);
    edges.points.push_back(p2);
  }
  return edges;
}

visualization_msgs::msg::Marker PoseGraphManager::visualizeLoopDetectionRadius(
    const geometry_msgs::msg::Point &latest_position) const {
  visualization_msgs::msg::Marker sphere;
  sphere.header.frame_id = map_frame_;
  sphere.id              = 100000;  // arbitrary number
  sphere.type            = visualization_msgs::msg::Marker::SPHERE;
  sphere.pose.position.x = latest_position.x;
  sphere.pose.position.y = latest_position.y;
  sphere.pose.position.z = latest_position.z;
  sphere.scale.x         = 2 * loop_detection_radius_;
  sphere.scale.y         = 2 * loop_detection_radius_;
  sphere.scale.z         = 2 * loop_detection_radius_;
  // Use transparanet cyan color
  sphere.color.r = 0.0;
  sphere.color.g = 0.824;
  sphere.color.b = 1.0;
  sphere.color.a = 0.5;

  return sphere;
}

void PoseGraphManager::saveFlagCallback(const std_msgs::msg::String::ConstSharedPtr &msg) {
  const std::filesystem::path save_dir =
      !msg->data.empty() ? std::filesystem::path(msg->data)
                         : std::filesystem::path(output_directory_);
  const auto keyframes = map_database_.snapshot();
  if (keyframes.empty()) {
    RCLCPP_WARN(this->get_logger(), "No keyframes available; skipping map product save");
    return;
  }

  try {
    if (save_in_kitti_format_) {
      const auto directory = map_product_writer_->writeKittiDataset(keyframes, save_dir);
      RCLCPP_INFO(this->get_logger(),
                  "Scans and poses saved in PCD, KITTI and TUM format below %s",
                  directory.c_str());
    }
    if (save_map_pcd_) {
      const auto path = map_product_writer_->writeMap(keyframes, save_dir, false);
      RCLCPP_INFO(this->get_logger(), "Accumulated map cloud saved to %s", path.c_str());
    }
  } catch (const std::exception &error) {
    RCLCPP_ERROR(this->get_logger(),
                 "Map product save request failed without changing the running map: %s",
                 error.what());
  }
}

// ----------------------------------------------------------------------

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  auto node = std::make_shared<PoseGraphManager>(options);

  // To allow timer callbacks to run concurrently using multiple threads
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
