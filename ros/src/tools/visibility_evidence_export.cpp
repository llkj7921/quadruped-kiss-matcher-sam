#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include "kiss_matcher_slam/core/map_product_loader.hpp"
#include "kiss_matcher_slam/core/visibility_map_filter.hpp"

namespace fs = std::filesystem;
int main(int argc, char** argv) {
  try {
    if (argc != 3) throw std::invalid_argument("usage: visibility_evidence_export MAP_PRODUCT NEW_OUTPUT_DIRECTORY");
    const fs::path input(argv[1]), output(argv[2]);
    if (fs::exists(output)) throw std::runtime_error("output directory already exists");
    const auto frames = kiss_matcher_slam::MapProductLoader::load(input);
    kiss_matcher_slam::VisibilityMapFilterConfig config;
    config.output_voxel_resolution_m = .05;
    config.capture_point_evidence = true;
    const auto result = kiss_matcher_slam::VisibilityMapFilter(config).filter(frames);
    if (result.point_evidence.size() != frames.size()) throw std::runtime_error("missing evidence");
    fs::create_directories(output);
    std::ofstream out(output / "point_evidence.tsv");
    if (!out) throw std::runtime_error("cannot write evidence");
    out << "# frame_id point_id free occupied occluded\n";
    for (std::size_t i = 0; i < frames.size(); ++i) {
      if (result.point_evidence[i].size() != frames[i].cloud.size()) throw std::runtime_error("evidence length mismatch");
      for (std::size_t j = 0; j < result.point_evidence[i].size(); ++j) {
        const auto& e = result.point_evidence[i][j];
        out << i << ' ' << j << ' ' << e.free << ' ' << e.occupied << ' ' << e.occluded << '\n';
      }
    }
    out.flush();
    if (!out) throw std::runtime_error("evidence write failed");
    std::ofstream report(output / "report.txt");
    report << "frames " << frames.size() << "\npoints " << result.input_points << "\ndynamic " << result.dynamic_points
           << "\nstatic " << result.static_points << "\nuncertain " << result.uncertain_points << "\n";
    report.flush();
    if (!report) throw std::runtime_error("report write failed");
    std::cout << "frames=" << frames.size() << " points=" << result.input_points << " dynamic=" << result.dynamic_points << '\n';
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
