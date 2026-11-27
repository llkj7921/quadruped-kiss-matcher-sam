#include "kiss_matcher_slam/core/visibility_map_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace kiss_matcher_slam {
namespace {
constexpr double kPi = 3.14159265358979323846;
struct Ray { Eigen::Vector3f direction; float range; };
using RayGrid = std::unordered_map<std::int64_t, std::vector<Ray>>;
using Evidence = VisibilityPointEvidence;

std::int64_t key(int azimuth, int elevation) {
  return (static_cast<std::int64_t>(azimuth) << 32) | static_cast<std::uint32_t>(elevation);
}
void voxelize(pcl::PointCloud<SlamPoint>& cloud, double resolution) {
  if (cloud.empty()) return;
  auto input = cloud.makeShared();
  pcl::VoxelGrid<SlamPoint> grid;
  grid.setInputCloud(input);
  grid.setLeafSize(resolution, resolution, resolution);
  grid.filter(cloud);
}
}  // namespace

VisibilityMapFilter::VisibilityMapFilter(VisibilityMapFilterConfig config) : config_(config) {
  if (!std::isfinite(config.angular_resolution_rad) || config.angular_resolution_rad < 0.001 ||
      config.angular_resolution_rad > 0.1 || !std::isfinite(config.range_margin_m) ||
      config.range_margin_m <= 0 || !std::isfinite(config.maximum_range_m) ||
      config.maximum_range_m <= 0 || !std::isfinite(config.minimum_time_separation_s) ||
      config.minimum_time_separation_s < 0 || !std::isfinite(config.output_voxel_resolution_m) ||
      config.output_voxel_resolution_m <= 0 || config.minimum_free_observations == 0 ||
      !std::isfinite(config.free_to_occupied_ratio) || config.free_to_occupied_ratio < 1)
    throw std::invalid_argument("invalid visibility filter configuration");
}

VisibilityMapFilterResult VisibilityMapFilter::filter(const std::vector<Keyframe>& frames) const {
  if (frames.empty()) throw std::invalid_argument("cannot filter an empty keyframe sequence");
  const double step = config_.angular_resolution_rad;
  const int azimuth_bins = static_cast<int>(std::ceil(2 * kPi / step));
  const double cosine_limit = std::cos(step);
  auto bins = [&](const Eigen::Vector3f& v) {
    int az = static_cast<int>(std::floor((std::atan2(v.y(), v.x()) + kPi) / step));
    int el = static_cast<int>(std::floor((std::atan2(v.z(), std::hypot(v.x(), v.y())) + kPi / 2) / step));
    return std::pair<int, int>{std::min(az, azimuth_bins - 1), el};
  };
  std::vector<pcl::PointCloud<SlamPoint>> world(frames.size());
  std::vector<std::vector<Evidence>> evidence(frames.size());
  VisibilityMapFilterResult result;
  for (std::size_t i = 0; i < frames.size(); ++i) {
    if (!frames[i].optimized_pose.allFinite() || !std::isfinite(frames[i].timestamp))
      throw std::invalid_argument("invalid keyframe pose or timestamp");
    pcl::transformPointCloud(frames[i].cloud, world[i], frames[i].optimized_pose);
    evidence[i].resize(world[i].size());
    result.input_points += world[i].size();
  }
  for (std::size_t ref = 0; ref < frames.size(); ++ref) {
    RayGrid grid;
    for (const auto& point : frames[ref].cloud) {
      Eigen::Vector3f v(point.x, point.y, point.z);
      const float distance = v.norm();
      if (!v.allFinite() || distance < 0.1f || distance > config_.maximum_range_m) continue;
      const auto [az, el] = bins(v);
      grid[key(az, el)].push_back({v / distance, distance});
    }
    const Eigen::Matrix4f inverse = frames[ref].optimized_pose.inverse().cast<float>();
    for (std::size_t src = 0; src < frames.size(); ++src) {
      if (src == ref || std::abs(frames[src].timestamp - frames[ref].timestamp) < config_.minimum_time_separation_s) continue;
      for (std::size_t j = 0; j < world[src].size(); ++j) {
        const auto& p = world[src][j];
        const Eigen::Vector4f global(p.x, p.y, p.z, 1);
        if (!global.allFinite()) continue;
        const Eigen::Vector3f v = (inverse * global).head<3>();
        const float distance = v.norm();
        if (!v.allFinite() || distance < 0.1f || distance > config_.maximum_range_m) continue;
        const Eigen::Vector3f direction = v / distance;
        const auto [az, el] = bins(v);
        float closest_cos = static_cast<float>(cosine_limit);
        float measured_range = 0;
        for (int da = -1; da <= 1; ++da) for (int de = -1; de <= 1; ++de) {
          int a = (az + da + azimuth_bins) % azimuth_bins;
          const auto it = grid.find(key(a, el + de));
          if (it == grid.end()) continue;
          for (const auto& ray : it->second) {
            const float dot = direction.dot(ray.direction);
            if (dot > closest_cos) { closest_cos = dot; measured_range = ray.range; }
          }
        }
        if (measured_range == 0) continue;
        const double margin = config_.range_margin_m + distance * step;
        auto& vote = evidence[src][j];
        if (measured_range > distance + margin) { ++vote.free; ++result.free_observations; }
        else if (measured_range < distance - margin) { ++vote.occluded; ++result.occluded_observations; }
        else { ++vote.occupied; ++result.occupied_observations; }
      }
    }
  }
  for (std::size_t i = 0; i < frames.size(); ++i) for (std::size_t j = 0; j < world[i].size(); ++j) {
    const auto& p = world[i][j];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(p.intensity)) continue;
    const auto& vote = evidence[i][j];
    if (vote.free >= config_.minimum_free_observations &&
        static_cast<double>(vote.free) >= config_.free_to_occupied_ratio * std::max<std::size_t>(1, vote.occupied)) {
      result.removed.push_back(p); ++result.dynamic_points;
    } else {
      result.retained.push_back(p);
      if (vote.occupied == 0) { result.uncertain.push_back(p); ++result.uncertain_points; }
      else ++result.static_points;
    }
  }
  voxelize(result.retained, config_.output_voxel_resolution_m);
  voxelize(result.removed, config_.output_voxel_resolution_m);
  voxelize(result.uncertain, config_.output_voxel_resolution_m);
  if (config_.capture_point_evidence) result.point_evidence = std::move(evidence);
  return result;
}
}  // namespace kiss_matcher_slam
