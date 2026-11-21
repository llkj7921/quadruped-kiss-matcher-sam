#include "kiss_matcher_slam/core/map_product_loader.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>

namespace kiss_matcher_slam {
namespace fs = std::filesystem;
namespace {

std::vector<double> readTimestampFile(const fs::path &path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open " + path.string());
  std::vector<double> timestamps;
  std::size_t id;
  double stamp;
  while (input >> id >> stamp) {
    if (id != timestamps.size()) throw std::runtime_error("non-contiguous keyframe IDs in " + path.string());
    timestamps.push_back(stamp);
  }
  if (!input.eof()) throw std::runtime_error("malformed timestamp file: " + path.string());
  return timestamps;
}

std::vector<Eigen::Matrix4d> readTumFile(const fs::path &path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open " + path.string());
  std::vector<Eigen::Matrix4d> poses;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line.front() == '#') continue;
    std::istringstream row(line);
    double stamp, x, y, z, qx, qy, qz, qw;
    if (!(row >> stamp >> x >> y >> z >> qx >> qy >> qz >> qw)) {
      throw std::runtime_error("malformed TUM trajectory: " + path.string());
    }
    Eigen::Quaterniond q(qw, qx, qy, qz);
    if (q.norm() < 1e-9) throw std::runtime_error("zero quaternion in " + path.string());
    q.normalize();
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    pose.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
    poses.push_back(pose);
  }
  return poses;
}

}  // namespace

std::vector<Keyframe> MapProductLoader::load(const fs::path &directory) {
  if (!fs::is_directory(directory)) throw std::runtime_error("map product directory does not exist: " + directory.string());
  const auto metadata = directory / "metadata.yaml";
  const auto timestamps_path = directory / "timestamps.txt";
  const auto trajectory_path = directory / "optimized_trajectory_tum.txt";
  const auto keyframes_dir = directory / "keyframes";
  if (!fs::exists(metadata) || !fs::exists(timestamps_path) || !fs::exists(trajectory_path) ||
      !fs::is_directory(keyframes_dir)) {
    throw std::runtime_error("incomplete map product: " + directory.string());
  }
  const auto timestamps = readTimestampFile(timestamps_path);
  const auto optimized = readTumFile(trajectory_path);
  if (timestamps.empty() || timestamps.size() != optimized.size()) {
    throw std::runtime_error("map product trajectory count does not match timestamps");
  }
  std::vector<Keyframe> keyframes;
  keyframes.reserve(timestamps.size());
  for (std::size_t id = 0; id < timestamps.size(); ++id) {
    char filename[32];
    std::snprintf(filename, sizeof(filename), "%06zu.pcd", id);
    Keyframe frame;
    frame.id = id;
    frame.timestamp = timestamps[id];
    frame.optimized_pose = optimized[id];
    frame.odometry_pose = optimized[id];
    const auto pcd_path = keyframes_dir / filename;
    if (pcl::io::loadPCDFile<SlamPoint>(pcd_path.string(), frame.cloud) < 0 || frame.cloud.empty()) {
      throw std::runtime_error("cannot read non-empty keyframe: " + pcd_path.string());
    }
    keyframes.push_back(std::move(frame));
  }
  return keyframes;
}

}  // namespace kiss_matcher_slam
