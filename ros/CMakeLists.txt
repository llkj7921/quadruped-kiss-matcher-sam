cmake_minimum_required(VERSION 3.22)
project(kiss_matcher_ros)

# Set a default build type if none is specified
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
    # Provide options for cmake-gui or ccmake
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
cmake_policy(SET CMP0074 NEW)

option(USE_ADDRESS_SANITIZER "Use address sanitizer" OFF)
if (USE_ADDRESS_SANITIZER)
    add_compile_options(-fsanitize=address -fsanitize-recover=address)
    add_link_options(-fsanitize=address -fsanitize-recover=address)
endif()

# oneTBB probes the assembler version and parses its text output. Some
# versions set only LANG=C for that probe, which is ineffective when the
# caller has LC_ALL=zh_CN. Set the configure-process environment before any
# third-party add_subdirectory call. This does not change the caller's shell.
set(ENV{LC_ALL} "C")
set(ENV{LANG} "C")
set(ENV{LANGUAGE} "C")

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../cpp/kiss_matcher/ AND NOT IS_SYMLINK ${CMAKE_CURRENT_SOURCE_DIR})
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cpp/kiss_matcher ${CMAKE_CURRENT_BINARY_DIR}/kiss_matcher)
else()
  #This is what kiss_icp does.
    #cmake_minimum_required(VERSION 3.18)
    #message(STATUS "Performing out-of-tree build, fetching KISS-Matcher v${CMAKE_PROJECT_VERSION} Release from Github")
    #include(FetchContent)
    #FetchContent_Declare(
    #  ext_kiss_matcher_core PREFIX kiss_matcher_core
    #  URL https://github.com/PRBonn/kiss-matcher/archive/refs/tags/v${CMAKE_PROJECT_VERSION}.tar.gz SOURCE_SUBDIR
    #      cpp/kiss_matcher)
    #FetchContent_MakeAvailable(ext_kiss_matcher_core)
endif()

# Load small_gicp
include(FetchContent)
# small_gicp uses the generic BUILD_TESTS cache key. Force its data-dependent
# upstream tests off; this package owns its tests through BUILD_TESTING below.
set(BUILD_TESTS OFF CACHE BOOL "Disable dependency tests" FORCE)
FetchContent_Declare(
  small_gicp
  GIT_REPOSITORY https://github.com/koide3/small_gicp
  # Verified with ROS 2 Humble, GCC 11 and both C/zh_CN configure locales.
  # Pin the immutable revision so upstream master cannot change our build.
  GIT_TAG fa0cfc983417c044b6036f71dfa448038bb9c820
)
FetchContent_MakeAvailable(small_gicp)
# CMake leaves a dependency's old CTestTestfile.cmake behind when BUILD_TESTS is
# switched from ON to OFF. Clear that generated manifest so `colcon test` only
# reports tests owned by this package even when reusing an existing build tree.
if(NOT BUILD_TESTS AND DEFINED small_gicp_BINARY_DIR)
  file(WRITE "${small_gicp_BINARY_DIR}/CTestTestfile.cmake" "")
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(slam_system_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

find_package(PCL REQUIRED)
include_directories(
    include
    src
    ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(GTSAM REQUIRED)

add_library(kiss_matcher_slam_core
  src/core/frame_and_transform_contract.cpp
  src/core/gps_altitude_manager.cpp
  src/core/incremental_map_builder.cpp
  src/core/keyframe_processor.cpp
  src/core/keyframe_selector.cpp
  src/core/loop_candidate_queue.cpp
  src/core/loop_constraint_processor.cpp
  src/core/localization_quality_monitor.cpp
  src/core/map_database.cpp
  src/core/map_product_loader.cpp
  src/core/mapping_runtime_state.cpp
  src/core/offline_dynamic_map_filter.cpp
  src/core/map_product_writer.cpp
  src/core/pose_graph_backend.cpp
  src/core/realtime_pose_tracker.cpp
  src/core/recovery_coordinator.cpp
  src/core/scan_context_loop_detector.cpp
  src/core/system_mode_manager.cpp
  src/core/visualization_state.cpp
  src/core/visibility_map_filter.cpp
)
target_include_directories(kiss_matcher_slam_core PUBLIC include ${PCL_INCLUDE_DIRS})
target_link_libraries(kiss_matcher_slam_core kiss_matcher::kiss_matcher_core ${PCL_LIBRARIES} gtsam)

################################################################################
add_executable(run_kiss_matcher src/run_kiss_matcher.cpp)
target_include_directories(run_kiss_matcher
    PUBLIC
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(run_kiss_matcher
    kiss_matcher::kiss_matcher_core
    ${PCL_LIBRARIES}
)

add_executable(registration_visualizer src/registration_visualizer.cpp)
ament_target_dependencies(registration_visualizer
  rclcpp
  tf2_ros
  sensor_msgs
  pcl_conversions
  PCL
)
target_link_libraries(registration_visualizer
  kiss_matcher::kiss_matcher_core
  ${PCL_LIBRARIES}
)

add_executable(kiss_matcher_sam src/slam/pose_graph_manager.cpp
                                src/slam/loop_closure.cpp
                                src/slam/loop_registration_coordinator.cpp)
ament_target_dependencies(kiss_matcher_sam
  rclcpp
  tf2_ros
  sensor_msgs
  nav_msgs
  visualization_msgs
  slam_system_interfaces
  pcl_conversions
  PCL
)

add_executable(kiss_matcher_relocalizer src/relocalizer.cpp
                                        src/slam/loop_closure.cpp)
ament_target_dependencies(kiss_matcher_relocalizer
  rclcpp tf2_ros sensor_msgs nav_msgs geometry_msgs std_msgs slam_system_interfaces pcl_conversions PCL
)
target_link_libraries(kiss_matcher_relocalizer
  kiss_matcher_slam_core kiss_matcher::kiss_matcher_core ${PCL_LIBRARIES} small_gicp gtsam
)

target_link_libraries(kiss_matcher_sam
  kiss_matcher_slam_core
  kiss_matcher::kiss_matcher_core
  ${PCL_LIBRARIES}
  small_gicp
  gtsam
)

add_executable(inter_frame_alignment src/inter_frame_alignment.cpp
                                     src/slam/loop_closure.cpp)
ament_target_dependencies(inter_frame_alignment
  rclcpp
  tf2_ros
  sensor_msgs
  nav_msgs
  visualization_msgs
  pcl_conversions
  PCL
)

target_link_libraries(inter_frame_alignment
  kiss_matcher::kiss_matcher_core
  ${PCL_LIBRARIES}
  small_gicp
  gtsam
)

add_executable(scan_context_benchmark src/tools/scan_context_benchmark.cpp)
target_include_directories(scan_context_benchmark PUBLIC include ${PCL_INCLUDE_DIRS})
target_link_libraries(scan_context_benchmark kiss_matcher_slam_core ${PCL_LIBRARIES})

add_executable(offline_dynamic_map_filter src/tools/offline_dynamic_map_filter.cpp)
target_include_directories(offline_dynamic_map_filter PUBLIC include ${PCL_INCLUDE_DIRS})
target_link_libraries(offline_dynamic_map_filter kiss_matcher_slam_core ${PCL_LIBRARIES})

add_executable(visibility_map_filter src/tools/visibility_map_filter.cpp)
target_include_directories(visibility_map_filter PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(visibility_map_filter kiss_matcher_slam_core ${PCL_LIBRARIES})

add_executable(visibility_evidence_export src/tools/visibility_evidence_export.cpp)
target_include_directories(visibility_evidence_export PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(visibility_evidence_export kiss_matcher_slam_core ${PCL_LIBRARIES})

add_executable(system_mode_manager src/system_mode_manager_node.cpp)
ament_target_dependencies(system_mode_manager rclcpp std_msgs std_srvs slam_system_interfaces)
target_link_libraries(system_mode_manager kiss_matcher_slam_core)

install(TARGETS
  kiss_matcher_slam_core
  registration_visualizer
  run_kiss_matcher
  kiss_matcher_sam
  kiss_matcher_relocalizer
  inter_frame_alignment
  scan_context_benchmark
  offline_dynamic_map_filter
  visibility_map_filter
  visibility_evidence_export
  system_mode_manager
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_relocalization_confirmation test/test_relocalization_confirmation.cpp)
  target_include_directories(test_relocalization_confirmation PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_relocalization_confirmation kiss_matcher_slam_core)
  ament_add_gtest(test_visibility_map_filter test/test_visibility_map_filter.cpp)
  target_include_directories(test_visibility_map_filter PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_visibility_map_filter kiss_matcher_slam_core ${PCL_LIBRARIES})
  ament_add_gtest(test_online_dynamic_filter test/test_online_dynamic_filter.cpp)
  target_include_directories(test_online_dynamic_filter PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_online_dynamic_filter ${PCL_LIBRARIES})
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_recovery_state_chain.py TIMEOUT 60)
  ament_add_gtest(test_gps_altitude_manager test/test_gps_altitude_manager.cpp)
  target_include_directories(test_gps_altitude_manager PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_gps_altitude_manager kiss_matcher_slam_core)

  ament_add_gtest(test_incremental_map_builder test/test_incremental_map_builder.cpp)
  target_include_directories(test_incremental_map_builder PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_incremental_map_builder kiss_matcher_slam_core ${PCL_LIBRARIES})

  ament_add_gtest(test_keyframe_selector test/test_keyframe_selector.cpp)
  target_include_directories(test_keyframe_selector PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_keyframe_selector kiss_matcher_slam_core ${PCL_LIBRARIES})

  ament_add_gtest(test_keyframe_processor test/test_keyframe_processor.cpp)
  target_include_directories(test_keyframe_processor PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_keyframe_processor kiss_matcher_slam_core ${PCL_LIBRARIES} gtsam)

  ament_add_gtest(test_loop_candidate_queue test/test_loop_candidate_queue.cpp)
  target_include_directories(test_loop_candidate_queue PRIVATE include)
  target_link_libraries(test_loop_candidate_queue kiss_matcher_slam_core)

  ament_add_gtest(test_loop_constraint_processor test/test_loop_constraint_processor.cpp)
  target_include_directories(test_loop_constraint_processor PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_loop_constraint_processor kiss_matcher_slam_core ${PCL_LIBRARIES} gtsam)

  ament_add_gtest(test_mapping_runtime_state test/test_mapping_runtime_state.cpp)
  target_include_directories(test_mapping_runtime_state PRIVATE include)
  target_link_libraries(test_mapping_runtime_state kiss_matcher_slam_core)

  ament_add_gtest(test_map_database test/test_map_database.cpp)
  target_include_directories(test_map_database PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_map_database kiss_matcher_slam_core ${PCL_LIBRARIES})

  ament_add_gtest(test_map_product_writer test/test_map_product_writer.cpp)
  target_include_directories(test_map_product_writer PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_map_product_writer kiss_matcher_slam_core ${PCL_LIBRARIES})

  ament_add_gtest(test_offline_dynamic_map_filter test/test_offline_dynamic_map_filter.cpp)
  target_include_directories(test_offline_dynamic_map_filter PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_offline_dynamic_map_filter kiss_matcher_slam_core ${PCL_LIBRARIES})

  ament_add_gtest(test_pose_graph_backend test/test_pose_graph_backend.cpp)
  target_include_directories(test_pose_graph_backend PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_pose_graph_backend kiss_matcher_slam_core gtsam)

  ament_add_gtest(test_realtime_pose_tracker test/test_realtime_pose_tracker.cpp)
  target_include_directories(test_realtime_pose_tracker PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_realtime_pose_tracker kiss_matcher_slam_core)

  ament_add_gtest(test_scan_context_loop_detector test/test_scan_context_loop_detector.cpp)
  target_include_directories(test_scan_context_loop_detector PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_scan_context_loop_detector kiss_matcher_slam_core ${PCL_LIBRARIES})

  ament_add_gtest(test_visualization_state test/test_visualization_state.cpp)
  target_include_directories(test_visualization_state PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_visualization_state kiss_matcher_slam_core)

  ament_add_gtest(test_system_mode_manager test/test_system_mode_manager.cpp)
  target_include_directories(test_system_mode_manager PRIVATE include)
  target_link_libraries(test_system_mode_manager kiss_matcher_slam_core)

  ament_add_gtest(test_localization_quality_monitor test/test_localization_quality_monitor.cpp)
  target_include_directories(test_localization_quality_monitor PRIVATE include)
  target_link_libraries(test_localization_quality_monitor kiss_matcher_slam_core)

  ament_add_gtest(test_recovery_coordinator test/test_recovery_coordinator.cpp)
  target_include_directories(test_recovery_coordinator PRIVATE include)
  target_link_libraries(test_recovery_coordinator kiss_matcher_slam_core)

  ament_add_gtest(test_frame_and_transform_contract test/test_frame_and_transform_contract.cpp)
  target_include_directories(test_frame_and_transform_contract PRIVATE include)
  target_link_libraries(test_frame_and_transform_contract kiss_matcher_slam_core)

  ament_add_gtest(test_loop_closure test/test_loop_closure.cpp src/slam/loop_closure.cpp)
  target_include_directories(test_loop_closure PRIVATE include ${PCL_INCLUDE_DIRS})
  ament_target_dependencies(test_loop_closure
    rclcpp geometry_msgs sensor_msgs nav_msgs pcl_conversions tf2_ros tf2_eigen tf2_geometry_msgs
  )
  target_link_libraries(test_loop_closure
    kiss_matcher::kiss_matcher_core ${PCL_LIBRARIES} small_gicp gtsam
  )

  ament_add_gtest(test_loop_registration_coordinator
    test/test_loop_registration_coordinator.cpp
    src/slam/loop_closure.cpp
    src/slam/loop_registration_coordinator.cpp)
  target_include_directories(test_loop_registration_coordinator PRIVATE include ${PCL_INCLUDE_DIRS})
  ament_target_dependencies(test_loop_registration_coordinator
    rclcpp geometry_msgs sensor_msgs nav_msgs pcl_conversions tf2_ros tf2_eigen tf2_geometry_msgs
  )
  target_link_libraries(test_loop_registration_coordinator
    kiss_matcher_slam_core kiss_matcher::kiss_matcher_core ${PCL_LIBRARIES} small_gicp gtsam
  )
endif()

install(DIRECTORY launch config rviz docs
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/validate_map_product.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
