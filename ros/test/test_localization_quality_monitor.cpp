#include <gtest/gtest.h>

#include "kiss_matcher_slam/core/localization_quality_monitor.hpp"

using kiss_matcher_slam::LocalizationHealth;
using kiss_matcher_slam::LocalizationQualityMonitor;
using kiss_matcher_slam::RawLocalizationQuality;

TEST(LocalizationQualityMonitor, DebouncesSuspectedAndRecoversOnStableGoodFrames) {
  LocalizationQualityMonitor monitor({2, 2, 3});
  EXPECT_EQ(monitor.update(RawLocalizationQuality::kSuspected), LocalizationHealth::kGood);
  EXPECT_EQ(monitor.update(RawLocalizationQuality::kSuspected), LocalizationHealth::kSuspected);
  EXPECT_EQ(monitor.update(RawLocalizationQuality::kGood), LocalizationHealth::kSuspected);
  EXPECT_EQ(monitor.update(RawLocalizationQuality::kGood), LocalizationHealth::kSuspected);
  EXPECT_EQ(monitor.update(RawLocalizationQuality::kGood), LocalizationHealth::kGood);
}

TEST(LocalizationQualityMonitor, EscalatesRepeatedBadFrames) {
  LocalizationQualityMonitor monitor({2, 2, 3});
  monitor.update(RawLocalizationQuality::kBad);
  EXPECT_EQ(monitor.update(RawLocalizationQuality::kBad), LocalizationHealth::kBad);
  monitor.reset();
  EXPECT_EQ(monitor.health(), LocalizationHealth::kGood);
}

TEST(LocalizationQualityMonitor, GapCanImmediatelyMarkSuspected) {
  LocalizationQualityMonitor monitor({2, 2, 3});
  EXPECT_EQ(monitor.markSuspected(), LocalizationHealth::kSuspected);
  EXPECT_EQ(monitor.update(RawLocalizationQuality::kGood), LocalizationHealth::kSuspected);
  EXPECT_EQ(monitor.update(RawLocalizationQuality::kGood), LocalizationHealth::kSuspected);
  EXPECT_EQ(monitor.update(RawLocalizationQuality::kGood), LocalizationHealth::kGood);
}
