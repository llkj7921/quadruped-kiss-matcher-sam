#pragma once

namespace kiss_matcher_slam {

enum class RawLocalizationQuality { kGood, kSuspected, kBad };
enum class LocalizationHealth { kGood, kSuspected, kBad };

struct LocalizationQualityMonitorConfig {
  int suspected_confirmations = 2;
  int bad_confirmations = 2;
  int recovery_confirmations = 3;
};

class LocalizationQualityMonitor {
 public:
  explicit LocalizationQualityMonitor(LocalizationQualityMonitorConfig config = {});
  LocalizationHealth update(RawLocalizationQuality quality);
  LocalizationHealth markSuspected();
  void reset();
  LocalizationHealth health() const noexcept { return health_; }

 private:
  LocalizationQualityMonitorConfig config_;
  LocalizationHealth health_ = LocalizationHealth::kGood;
  int suspected_count_ = 0;
  int bad_count_ = 0;
  int good_count_ = 0;
};

}  // namespace kiss_matcher_slam
