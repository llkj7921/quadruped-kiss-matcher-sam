#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <pcl/io/pcd_io.h>
#include "kiss_matcher_slam/core/map_product_loader.hpp"
#include "kiss_matcher_slam/core/visibility_map_filter.hpp"

namespace fs = std::filesystem;
int main(int argc, char** argv) {
  try {
    if (argc != 3 && argc != 5) throw std::invalid_argument("usage: visibility_map_filter MAP_PRODUCT NEW_OUTPUT_DIRECTORY [--output-voxel METERS]");
    const fs::path input(argv[1]), output(argv[2]);
    if (fs::exists(output)) throw std::runtime_error("output directory already exists");
    const auto frames = kiss_matcher_slam::MapProductLoader::load(input);
    kiss_matcher_slam::VisibilityMapFilterConfig config;
    if (argc == 5) {
      if (std::string(argv[3]) != "--output-voxel") throw std::invalid_argument("unknown option: " + std::string(argv[3]));
      std::size_t consumed = 0;
      config.output_voxel_resolution_m = std::stod(argv[4], &consumed);
      if (consumed != std::string(argv[4]).size()) throw std::invalid_argument("invalid output voxel value");
    }
    const auto result = kiss_matcher_slam::VisibilityMapFilter(config).filter(frames);
    if (result.retained.empty()) throw std::runtime_error("filter produced an empty map");
    fs::create_directories(output);
    const auto write = [&](const char* name, const auto& cloud) {
      if (!cloud.empty() && pcl::io::savePCDFileBinary((output / name).string(), cloud) < 0)
        throw std::runtime_error("failed to save point cloud");
    };
    write("clean_map.pcd", result.retained);
    write("removed_points.pcd", result.removed);
    write("uncertain_points.pcd", result.uncertain);
    std::ofstream report(output / "report.yaml");
    if (!report) throw std::runtime_error("failed to create report");
    report << "method: approximate_ray_visibility\ninput_session: " << fs::absolute(input).string()
           << "\nkeyframes: " << frames.size() << "\ninput_points: " << result.input_points
           << "\nstatic_points: " << result.static_points << "\ndynamic_points: " << result.dynamic_points
           << "\nuncertain_points: " << result.uncertain_points
           << "\nfree_observations: " << result.free_observations
           << "\noccupied_observations: " << result.occupied_observations
           << "\noccluded_observations: " << result.occluded_observations
           << "\noutput_points: " << result.retained.size()
           << "\nangular_resolution_rad: " << config.angular_resolution_rad
           << "\nrange_margin_m: " << config.range_margin_m
           << "\nmaximum_range_m: " << config.maximum_range_m
           << "\nminimum_time_separation_s: " << config.minimum_time_separation_s
           << "\nminimum_free_observations: " << config.minimum_free_observations
           << "\nfree_to_occupied_ratio: " << config.free_to_occupied_ratio
           << "\noutput_voxel_resolution_m: " << config.output_voxel_resolution_m << "\n";
    if (!report) throw std::runtime_error("failed to write report");
    std::cout << "input=" << result.input_points << " removed=" << result.dynamic_points
              << " uncertain=" << result.uncertain_points << " output=" << result.retained.size() << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n'; return 1;
  }
}
