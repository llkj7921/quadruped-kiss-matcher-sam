#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <unistd.h>

#include "kiss_matcher_slam/core/map_product_loader.hpp"
#include "kiss_matcher_slam/core/map_product_writer.hpp"

namespace {
namespace fs = std::filesystem;

kiss_matcher_slam::Keyframe makeKeyframe(const std::size_t id,
                                         const double timestamp,
                                         const double x) {
  kiss_matcher_slam::Keyframe keyframe;
  keyframe.id = id;
  keyframe.timestamp = timestamp;
  keyframe.odometry_pose(0, 3) = x;
  keyframe.optimized_pose(0, 3) = x;
  kiss_matcher_slam::SlamPoint point;
  point.x = 0.0f;
  point.y = 0.0f;
  point.z = 0.0f;
  point.intensity = static_cast<float>(id);
  keyframe.cloud.push_back(point);
  return keyframe;
}

std::size_t lineCount(const fs::path &path) {
  std::ifstream input(path);
  std::size_t count = 0;
  std::string line;
  while (std::getline(input, line)) ++count;
  return count;
}

class MapProductWriterTest : public ::testing::Test {
 protected:
  void SetUp() override {
    root_ = fs::temp_directory_path() /
            ("kiss_matcher_map_product_test_" + std::to_string(::getpid()));
    fs::remove_all(root_);
    config_.output_directory = root_;
    config_.sequence = "test_sequence";
    config_.map_frame = "map";
    config_.base_frame = "base_link";
    config_.save_voxel_resolution = 0.1;
    config_.keyframe_translation_threshold_m = 1.5;
    config_.keyframe_rotation_threshold_rad = 0.2;
    config_.save_keyframes_incrementally = true;
    config_.overwrite_existing = true;
  }

  void TearDown() override { fs::remove_all(root_); }

  fs::path root_;
  kiss_matcher_slam::MapProductConfig config_;
};

TEST_F(MapProductWriterTest, RejectsUnsafeOrInvalidConfiguration) {
  auto invalid = config_;
  invalid.sequence = "../escape";
  EXPECT_THROW((void)kiss_matcher_slam::MapProductWriter{invalid}, std::invalid_argument);

  invalid = config_;
  invalid.save_voxel_resolution = 0.0;
  EXPECT_THROW((void)kiss_matcher_slam::MapProductWriter{invalid}, std::invalid_argument);

  invalid = config_;
  invalid.keyframe_rotation_threshold_rad =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((void)kiss_matcher_slam::MapProductWriter{invalid}, std::invalid_argument);
}

TEST_F(MapProductWriterTest, WritesIncrementalSessionAndTruncatesPreviousRun) {
  kiss_matcher_slam::MapProductWriter writer(config_);
  writer.initialize();
  writer.appendKeyframe(makeKeyframe(0, 10.0, 0.0));

  const fs::path directory = writer.sequenceDirectory();
  EXPECT_TRUE(fs::exists(directory / "metadata.yaml"));
  EXPECT_TRUE(fs::exists(directory / "keyframes/000000.pcd"));
  EXPECT_EQ(lineCount(directory / "timestamps.txt"), 1U);
  EXPECT_EQ(lineCount(directory / "odometry_tum.txt"), 1U);

  writer.initialize();
  EXPECT_EQ(lineCount(directory / "timestamps.txt"), 0U);
  EXPECT_EQ(lineCount(directory / "odometry_tum.txt"), 0U);
  EXPECT_FALSE(fs::exists(directory / "keyframes/000000.pcd"));
}

TEST_F(MapProductWriterTest, RefusesToOverwriteExistingSessionByDefault) {
  config_.overwrite_existing = false;
  kiss_matcher_slam::MapProductWriter first(config_);
  first.initialize();
  first.appendKeyframe(makeKeyframe(0, 10.0, 0.0));

  kiss_matcher_slam::MapProductWriter second(config_);
  EXPECT_THROW(second.initialize(), std::runtime_error);
  EXPECT_TRUE(fs::exists(first.sequenceDirectory() / "keyframes/000000.pcd"));
}

TEST_F(MapProductWriterTest, RequiresInitializationBeforeIncrementalWrite) {
  kiss_matcher_slam::MapProductWriter writer(config_);
  EXPECT_THROW(writer.appendKeyframe(makeKeyframe(0, 10.0, 0.0)), std::logic_error);
}

TEST_F(MapProductWriterTest, RejectsDuplicateAndSkippedIncrementalIds) {
  kiss_matcher_slam::MapProductWriter writer(config_);
  writer.initialize();
  writer.appendKeyframe(makeKeyframe(0, 10.0, 0.0));
  EXPECT_THROW(writer.appendKeyframe(makeKeyframe(0, 11.0, 1.0)), std::invalid_argument);
  EXPECT_THROW(writer.appendKeyframe(makeKeyframe(2, 12.0, 2.0)), std::invalid_argument);
  writer.appendKeyframe(makeKeyframe(1, 11.0, 1.0));
  EXPECT_EQ(lineCount(writer.sequenceDirectory() / "timestamps.txt"), 2U);
}

TEST_F(MapProductWriterTest, WritesCompleteOptimizedTrajectory) {
  kiss_matcher_slam::MapProductWriter writer(config_);
  writer.initialize();
  const std::vector<kiss_matcher_slam::Keyframe> keyframes{
      makeKeyframe(0, 10.0, 0.0), makeKeyframe(1, 11.0, 2.0)};
  std::vector<Eigen::Matrix4d> poses{keyframes[0].optimized_pose,
                                     keyframes[1].optimized_pose};

  writer.writeOptimizedTrajectory(keyframes, poses);
  EXPECT_EQ(lineCount(writer.sequenceDirectory() / "optimized_trajectory_tum.txt"), 3U);

  poses.pop_back();
  EXPECT_THROW(writer.writeOptimizedTrajectory(keyframes, poses), std::invalid_argument);
}

TEST_F(MapProductWriterTest, LoadsCompleteMapProductForRelocalization) {
  kiss_matcher_slam::MapProductWriter writer(config_);
  writer.initialize();
  const std::vector<kiss_matcher_slam::Keyframe> keyframes{
      makeKeyframe(0, 10.0, 0.0), makeKeyframe(1, 11.0, 2.0)};
  writer.appendKeyframe(keyframes[0]);
  writer.appendKeyframe(keyframes[1]);
  const std::vector<Eigen::Matrix4d> optimized{keyframes[0].optimized_pose,
                                               keyframes[1].optimized_pose};
  writer.writeOptimizedTrajectory(keyframes, optimized);

  const auto loaded =
      kiss_matcher_slam::MapProductLoader::load(writer.sequenceDirectory());
  ASSERT_EQ(loaded.size(), 2U);
  EXPECT_EQ(loaded[0].id, 0U);
  EXPECT_DOUBLE_EQ(loaded[1].timestamp, 11.0);
  EXPECT_NEAR(loaded[1].optimized_pose(0, 3), 2.0, 1e-9);
  ASSERT_EQ(loaded[1].cloud.size(), 1U);
}

TEST_F(MapProductWriterTest, BuildsVoxelizedMapFromOptimizedPoses) {
  kiss_matcher_slam::MapProductWriter writer(config_);
  const std::vector<kiss_matcher_slam::Keyframe> keyframes{
      makeKeyframe(0, 10.0, 0.0), makeKeyframe(1, 11.0, 2.0)};
  const fs::path map_path = writer.writeMap(keyframes, true);

  pcl::PointCloud<kiss_matcher_slam::SlamPoint> cloud;
  ASSERT_EQ(pcl::io::loadPCDFile(map_path.string(), cloud), 0);
  ASSERT_EQ(cloud.size(), 2U);
  EXPECT_NEAR(cloud.front().x, 0.0, 1e-6);
  EXPECT_NEAR(cloud.back().x, 2.0, 1e-6);
}

TEST_F(MapProductWriterTest, WritesKittiAndTumSnapshotWithoutDeletingOtherProducts) {
  kiss_matcher_slam::MapProductWriter writer(config_);
  const std::vector<kiss_matcher_slam::Keyframe> keyframes{
      makeKeyframe(0, 10.0, 0.0), makeKeyframe(1, 11.0, 2.0)};
  const fs::path directory = writer.writeKittiDataset(keyframes, root_ / "export");

  EXPECT_TRUE(fs::exists(directory / "scans/000000.pcd"));
  EXPECT_TRUE(fs::exists(directory / "scans/000001.pcd"));
  EXPECT_EQ(lineCount(directory / "poses_kitti.txt"), 2U);
  EXPECT_EQ(lineCount(directory / "poses_tum.txt"), 3U);
}

TEST_F(MapProductWriterTest, RefusesToOverwriteKittiScansWithoutPermission) {
  config_.overwrite_existing = false;
  kiss_matcher_slam::MapProductWriter writer(config_);
  const std::vector<kiss_matcher_slam::Keyframe> keyframes{
      makeKeyframe(0, 10.0, 0.0), makeKeyframe(1, 11.0, 2.0)};
  writer.writeKittiDataset(keyframes, root_ / "export");
  EXPECT_THROW(writer.writeKittiDataset(keyframes, root_ / "export"), std::runtime_error);
}

}  // namespace
