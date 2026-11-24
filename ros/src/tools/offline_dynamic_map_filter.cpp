#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include <pcl/io/pcd_io.h>

#include "kiss_matcher_slam/core/map_product_loader.hpp"
#include "kiss_matcher_slam/core/offline_dynamic_map_filter.hpp"

namespace fs = std::filesystem;

namespace {

struct Arguments {
  fs::path session_directory;
  fs::path output_path;
  fs::path report_path;
  kiss_matcher_slam::OfflineDynamicMapFilterConfig config;
};

void printUsage(const char* executable) {
  std::cerr << "Usage: " << executable
            << " <map_product_directory> [--output PATH] [--report PATH]"
               " [--observation-voxel METERS] [--output-voxel METERS]"
               " [--min-observations COUNT] [--min-duration SECONDS]\n";
}

std::string requireValue(const int argc, char** argv, int& index) {
  if (++index >= argc) throw std::invalid_argument("missing value after " + std::string(argv[index - 1]));
  return argv[index];
}

Arguments parseArguments(const int argc, char** argv) {
  if (argc < 2) throw std::invalid_argument("map product directory is required");
  Arguments arguments;
  arguments.session_directory = argv[1];
  for (int i = 2; i < argc; ++i) {
    const std::string option = argv[i];
    if (option == "--output") {
      arguments.output_path = requireValue(argc, argv, i);
    } else if (option == "--report") {
      arguments.report_path = requireValue(argc, argv, i);
    } else if (option == "--observation-voxel") {
      arguments.config.observation_voxel_resolution = std::stod(requireValue(argc, argv, i));
    } else if (option == "--output-voxel") {
      arguments.config.output_voxel_resolution = std::stod(requireValue(argc, argv, i));
    } else if (option == "--min-observations") {
      const auto value = std::stoll(requireValue(argc, argv, i));
      if (value <= 0) throw std::invalid_argument("min observations must be positive");
      arguments.config.minimum_observations = static_cast<std::size_t>(value);
    } else if (option == "--min-duration") {
      arguments.config.minimum_observation_duration_s = std::stod(requireValue(argc, argv, i));
    } else {
      throw std::invalid_argument("unknown option: " + option);
    }
  }
  if (arguments.output_path.empty()) {
    arguments.output_path = arguments.session_directory / "clean_map.pcd";
  }
  if (arguments.report_path.empty()) {
    arguments.report_path = arguments.session_directory / "clean_map_report.yaml";
  }
  return arguments;
}

void writeReport(const fs::path& path,
                 const Arguments& arguments,
                 const kiss_matcher_slam::OfflineDynamicMapFilterResult& result) {
  std::ofstream report(path, std::ios::trunc);
  if (!report) throw std::runtime_error("cannot write report: " + path.string());
  report << "method: keyframe_voxel_persistence\n"
         << "input_session: " << fs::absolute(arguments.session_directory).string() << "\n"
         << "output_map: " << fs::absolute(arguments.output_path).string() << "\n"
         << "observation_voxel_resolution_m: "
         << arguments.config.observation_voxel_resolution << "\n"
         << "output_voxel_resolution_m: " << arguments.config.output_voxel_resolution << "\n"
         << "minimum_observations: " << arguments.config.minimum_observations << "\n"
         << "minimum_observation_duration_s: "
         << arguments.config.minimum_observation_duration_s << "\n"
         << "input_keyframes: " << result.input_keyframes << "\n"
         << "input_points: " << result.input_points << "\n"
         << "observed_voxels: " << result.observed_voxels << "\n"
         << "persistent_voxels: " << result.persistent_voxels << "\n"
         << "retained_points_before_output_voxelization: "
         << result.retained_points_before_output_voxelization << "\n"
         << "output_points: " << result.cloud.size() << "\n";
  if (!report) throw std::runtime_error("failed while writing report: " + path.string());
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const auto arguments = parseArguments(argc, argv);
    const auto keyframes =
        kiss_matcher_slam::MapProductLoader::load(arguments.session_directory);
    const kiss_matcher_slam::OfflineDynamicMapFilter filter(arguments.config);
    const auto result = filter.filter(keyframes);
    if (result.cloud.empty()) {
      throw std::runtime_error(
          "filter produced an empty map; reduce min-observations/min-duration or increase observation voxel size");
    }
    if (!arguments.output_path.parent_path().empty()) {
      fs::create_directories(arguments.output_path.parent_path());
    }
    if (pcl::io::savePCDFileBinary(arguments.output_path.string(), result.cloud) < 0) {
      throw std::runtime_error("cannot write output map: " + arguments.output_path.string());
    }
    if (!arguments.report_path.parent_path().empty()) {
      fs::create_directories(arguments.report_path.parent_path());
    }
    writeReport(arguments.report_path, arguments, result);
    std::cout << "Filtered map written to " << arguments.output_path << "\n"
              << "Input points: " << result.input_points
              << ", retained before output voxelization: "
              << result.retained_points_before_output_voxelization
              << ", output points: " << result.cloud.size() << "\n"
              << "Persistent voxels: " << result.persistent_voxels << " / "
              << result.observed_voxels << "\n";
    return EXIT_SUCCESS;
  } catch (const std::exception& error) {
    printUsage(argv[0]);
    std::cerr << "Error: " << error.what() << "\n";
    return EXIT_FAILURE;
  }
}
