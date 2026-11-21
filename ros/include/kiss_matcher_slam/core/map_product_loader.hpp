#pragma once

#include <filesystem>
#include <vector>

#include "kiss_matcher_slam/core/keyframe.hpp"

namespace kiss_matcher_slam {

// Reads the complete, atomic map product written by MapProductWriter. A map
// is deliberately never represented by its final PCD alone: relocalization
// needs the individual LiDAR-frame keyframes and their optimized map poses.
class MapProductLoader {
 public:
  static std::vector<Keyframe> load(const std::filesystem::path &directory);
};

}  // namespace kiss_matcher_slam
