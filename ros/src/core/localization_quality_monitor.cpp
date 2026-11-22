#include "kiss_matcher_slam/core/localization_quality_monitor.hpp"

#include <stdexcept>

namespace kiss_matcher_slam {

LocalizationQualityMonitor::LocalizationQualityMonitor(
    const LocalizationQualityMonitorConfig config)
    : config_(config) {
  if (config_.suspected_confirmations <= 0 || config_.bad_confirmations <= 0 ||
      config_.recovery_confirmations <= 0) {
    throw std::invalid_argument("localization quality confirmation counts must be positive");
  }
}

LocalizationHealth LocalizationQualityMonitor::update(
    const RawLocalizationQuality quality) {
  suspected_count_ = quality == RawLocalizationQuality::kSuspected
                         ? suspected_count_ + 1
                         : 0;
  bad_count_ = quality == RawLocalizationQuality::kBad ? bad_count_ + 1 : 0;
  good_count_ = quality == RawLocalizationQuality::kGood ? good_count_ + 1 : 0;

  if (health_ == LocalizationHealth::kGood) {
    if (bad_count_ >= config_.bad_confirmations) {
      health_ = LocalizationHealth::kBad;
    } else if (suspected_count_ >= config_.suspected_confirmations) {
      health_ = LocalizationHealth::kSuspected;
    }
  } else if (health_ == LocalizationHealth::kSuspected) {
    if (bad_count_ >= config_.bad_confirmations) health_ = LocalizationHealth::kBad;
    if (good_count_ >= config_.recovery_confirmations) health_ = LocalizationHealth::kGood;
  }
  return health_;
}

LocalizationHealth LocalizationQualityMonitor::markSuspected() {
  if (health_ == LocalizationHealth::kGood) health_ = LocalizationHealth::kSuspected;
  suspected_count_ = bad_count_ = good_count_ = 0;
  return health_;
}

void LocalizationQualityMonitor::reset() {
  health_ = LocalizationHealth::kGood;
  suspected_count_ = bad_count_ = good_count_ = 0;
}

}  // namespace kiss_matcher_slam
