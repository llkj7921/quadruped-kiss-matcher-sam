#pragma once

#include <cstddef>
#include <Eigen/Core>
#include "kiss_matcher_slam/core/frame_and_transform_contract.hpp"

namespace kiss_matcher_slam {

// Only consecutive successful geometric queries may confirm a localization.
class RelocalizationConfirmation {
 public:
  void reset() noexcept { have_candidate_ = false; count_ = 0; }

  int observe(std::size_t candidate, const Eigen::Matrix4d& transform,
              double translation_tolerance, double rotation_tolerance) {
    if (!have_candidate_ || candidate != candidate_ ||
        !transformsConsistent(transform, transform_, translation_tolerance,
                              rotation_tolerance)) {
      count_ = 1;
    } else {
      ++count_;
    }
    candidate_ = candidate;
    transform_ = transform;
    have_candidate_ = true;
    return count_;
  }

 private:
  bool have_candidate_ = false;
  int count_ = 0;
  std::size_t candidate_ = 0;
  Eigen::Matrix4d transform_ = Eigen::Matrix4d::Identity();
};

}  // namespace kiss_matcher_slam
