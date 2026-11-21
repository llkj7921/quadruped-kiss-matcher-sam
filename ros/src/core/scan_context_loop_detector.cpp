#include "kiss_matcher_slam/core/scan_context_loop_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <kiss_matcher/kdtree/nanoflann.hpp>

namespace kiss_matcher_slam {
namespace {
constexpr double kTwoPi = 6.28318530717958647692;
}

class RingKeyDataset {
 public:
  RingKeyDataset(const std::vector<Eigen::VectorXf> &ring_keys,
                 std::vector<std::size_t> keyframe_ids)
      : ring_keys_(ring_keys), keyframe_ids_(std::move(keyframe_ids)) {}

  std::size_t kdtree_get_point_count() const { return keyframe_ids_.size(); }
  float kdtree_get_pt(const std::size_t index, const std::size_t dimension) const {
    return ring_keys_[keyframe_ids_[index]](static_cast<Eigen::Index>(dimension));
  }
  template <class BoundingBox>
  bool kdtree_get_bbox(BoundingBox &) const {
    return false;
  }
  std::size_t keyframeId(const std::size_t index) const { return keyframe_ids_[index]; }
  const std::vector<std::size_t> &keyframeIds() const { return keyframe_ids_; }
  void append(const std::vector<std::size_t> &keyframe_ids, const std::size_t begin) {
    keyframe_ids_.insert(keyframe_ids_.end(), keyframe_ids.begin() + begin, keyframe_ids.end());
  }

 private:
  const std::vector<Eigen::VectorXf> &ring_keys_;
  std::vector<std::size_t> keyframe_ids_;
};

using RingKeyDistance = kiss_matcher::L2_Simple_Adaptor<float, RingKeyDataset, float>;
using RingKeyTree =
    kiss_matcher::KDTreeSingleIndexDynamicAdaptor<RingKeyDistance, RingKeyDataset, -1, std::size_t>;

class RingKeySearchIndex {
 public:
  RingKeySearchIndex(const std::vector<Eigen::VectorXf> &ring_keys,
                     std::vector<std::size_t> keyframe_ids,
                     const int dimensions)
      : dataset_(ring_keys, std::move(keyframe_ids)),
        tree_(dimensions, dataset_, kiss_matcher::KDTreeSingleIndexAdaptorParams(10)) {
  }

  void synchronize(const std::vector<std::size_t> &keyframe_ids) {
    const auto &indexed_ids = dataset_.keyframeIds();
    if (keyframe_ids.size() < indexed_ids.size() ||
        !std::equal(indexed_ids.begin(), indexed_ids.end(), keyframe_ids.begin())) {
      throw std::invalid_argument("Scan Context eligible Ring Key history changed unexpectedly");
    }
    if (keyframe_ids.size() == indexed_ids.size()) return;
    const std::size_t first_new = indexed_ids.size();
    dataset_.append(keyframe_ids, first_new);
    tree_.addPoints(first_new, keyframe_ids.size() - 1);
  }

  std::size_t size() const { return dataset_.keyframeIds().size(); }

  std::vector<std::size_t> nearest(const Eigen::VectorXf &query, const std::size_t count) const {
    std::vector<std::size_t> indices(count);
    std::vector<float> squared_distances(count);
    kiss_matcher::KNNResultSet<float, std::size_t> result(count);
    result.init(indices.data(), squared_distances.data());
    tree_.findNeighbors(result, query.data(), kiss_matcher::SearchParams());
    const std::size_t found = result.size();
    if (found != count) {
      throw std::runtime_error("Scan Context Ring Key KD-tree returned too few candidates");
    }
    for (auto &index : indices) index = dataset_.keyframeId(index);
    return indices;
  }

 private:
  RingKeyDataset dataset_;
  RingKeyTree tree_;
};

ScanContextLoopDetector::ScanContextLoopDetector(ScanContextConfig config)
    : config_(std::move(config)) {
  if (config_.num_rings <= 0 || config_.num_sectors <= 0 || config_.exclude_recent < 0 ||
      config_.num_candidates <= 0 || !std::isfinite(config_.max_radius_m) ||
      config_.max_radius_m <= 0.0 || !std::isfinite(config_.distance_threshold) ||
      config_.distance_threshold < 0.0 || config_.distance_threshold > 1.0 ||
      !std::isfinite(config_.minimum_time_separation_s) ||
      config_.minimum_time_separation_s < 0.0 ||
      !std::isfinite(config_.sector_search_ratio) || config_.sector_search_ratio <= 0.0 ||
      config_.sector_search_ratio > 1.0) {
    throw std::invalid_argument("invalid Scan Context configuration");
  }
}

ScanContextLoopDetector::~ScanContextLoopDetector() = default;

Eigen::MatrixXf ScanContextLoopDetector::makeDescriptor(
    const pcl::PointCloud<SlamPoint> &cloud) const {
  Eigen::MatrixXf descriptor =
      Eigen::MatrixXf::Constant(config_.num_rings, config_.num_sectors, -1000.0f);
  const double ring_step = config_.max_radius_m / static_cast<double>(config_.num_rings);
  const double sector_step = kTwoPi / static_cast<double>(config_.num_sectors);

  for (const auto &point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    const double radius = std::hypot(point.x, point.y);
    if (radius < 1e-3 || radius > config_.max_radius_m) continue;
    double angle = std::atan2(point.y, point.x);
    if (angle < 0.0) angle += kTwoPi;
    const int ring = std::min(config_.num_rings - 1, static_cast<int>(radius / ring_step));
    const int sector =
        std::min(config_.num_sectors - 1, static_cast<int>(angle / sector_step));
    descriptor(ring, sector) = std::max(descriptor(ring, sector), point.z);
  }
  return descriptor.unaryExpr([](const float value) { return value < -999.0f ? 0.0f : value; });
}

Eigen::VectorXf ScanContextLoopDetector::makeRingKey(const Eigen::MatrixXf &descriptor) const {
  return descriptor.rowwise().mean();
}

Eigen::VectorXf ScanContextLoopDetector::makeSectorKey(const Eigen::MatrixXf &descriptor) const {
  return descriptor.colwise().mean().transpose();
}

int ScanContextLoopDetector::fastAlignUsingSectorKey(const Eigen::VectorXf &query,
                                                      const Eigen::VectorXf &candidate) const {
  int best_shift = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (int shift = 0; shift < config_.num_sectors; ++shift) {
    double squared_distance = 0.0;
    for (int sector = 0; sector < config_.num_sectors; ++sector) {
      const double difference =
          query(sector) - candidate((sector + shift) % config_.num_sectors);
      squared_distance += difference * difference;
    }
    if (squared_distance < best_distance) {
      best_distance = squared_distance;
      best_shift = shift;
    }
  }
  return best_shift;
}

ScanContextLoopDetector::DistanceResult ScanContextLoopDetector::descriptorDistance(
    const Eigen::MatrixXf &query,
    const Eigen::MatrixXf &candidate,
    const int coarse_shift,
    std::size_t &shift_evaluations) const {
  DistanceResult best;
  const int search_radius = static_cast<int>(std::round(
      0.5 * config_.sector_search_ratio * static_cast<double>(config_.num_sectors)));
  std::vector<int> search_shifts;
  search_shifts.reserve(static_cast<std::size_t>(2 * search_radius + 1));
  for (int offset = -search_radius; offset <= search_radius; ++offset) {
    search_shifts.push_back((coarse_shift + offset + config_.num_sectors) % config_.num_sectors);
  }
  std::sort(search_shifts.begin(), search_shifts.end());
  search_shifts.erase(std::unique(search_shifts.begin(), search_shifts.end()),
                      search_shifts.end());

  for (const int shift : search_shifts) {
    ++shift_evaluations;
    double similarity_sum = 0.0;
    int valid_columns = 0;
    for (int col = 0; col < config_.num_sectors; ++col) {
      const auto query_column = query.col(col);
      const auto candidate_column = candidate.col((col + shift) % config_.num_sectors);
      const double denominator = query_column.norm() * candidate_column.norm();
      if (denominator > 1e-6) {
        similarity_sum += query_column.dot(candidate_column) / denominator;
        ++valid_columns;
      }
    }
    if (valid_columns == 0) continue;
    const double distance = 1.0 - similarity_sum / static_cast<double>(valid_columns);
    if (distance < best.distance) best = {distance, shift};
  }
  return best;
}

void ScanContextLoopDetector::appendMissingDescriptors(
    const std::vector<Keyframe> &keyframes) {
  while (descriptors_.size() < keyframes.size()) {
    const auto index = descriptors_.size();
    if (keyframes[index].id != index) {
      throw std::invalid_argument("Scan Context keyframes must have contiguous IDs");
    }
    auto descriptor = makeDescriptor(keyframes[index].cloud);
    ring_keys_.push_back(makeRingKey(descriptor));
    sector_keys_.push_back(makeSectorKey(descriptor));
    descriptors_.push_back(std::move(descriptor));
  }
}

std::optional<PlaceCandidate> ScanContextLoopDetector::detect(
    const Keyframe &query, const std::vector<Keyframe> &keyframes) {
  using Clock = std::chrono::steady_clock;
  const auto total_start = Clock::now();
  last_stats_ = {};
  last_stats_.history_size = keyframes.size();
  if (keyframes.empty()) {
    last_stats_.total_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
    return std::nullopt;
  }
  const auto descriptor_count_before = descriptors_.size();
  const auto descriptor_start = Clock::now();
  appendMissingDescriptors(keyframes);
  last_stats_.descriptor_build_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - descriptor_start).count();
  last_stats_.descriptors_built = descriptors_.size() - descriptor_count_before;

  if (query.id >= descriptors_.size() || query.id >= keyframes.size() ||
      query.id <= static_cast<std::size_t>(config_.exclude_recent)) {
    last_stats_.total_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
    return std::nullopt;
  }

  const auto retrieval_start = Clock::now();

  std::vector<std::size_t> eligible_ids;
  const std::size_t search_end = query.id - static_cast<std::size_t>(config_.exclude_recent);
  for (std::size_t id = 0; id < search_end; ++id) {
    if (query.timestamp - keyframes[id].timestamp < config_.minimum_time_separation_s) continue;
    eligible_ids.push_back(id);
  }
  last_stats_.eligible_candidates = eligible_ids.size();
  if (eligible_ids.empty()) {
    last_stats_.retrieval_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - retrieval_start).count();
    last_stats_.total_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
    return std::nullopt;
  }

  if (!ring_key_index_) {
    ring_key_index_ = std::make_unique<RingKeySearchIndex>(
        ring_keys_, eligible_ids, config_.num_rings);
  } else {
    ring_key_index_->synchronize(eligible_ids);
  }
  last_stats_.indexed_candidates = ring_key_index_->size();
  const std::size_t indexed_candidate_count =
      std::min(ring_key_index_->size(), static_cast<std::size_t>(config_.num_candidates));
  last_stats_.shortlisted_candidates = indexed_candidate_count;
  const auto candidate_ids =
      ring_key_index_->nearest(ring_keys_[query.id], indexed_candidate_count);

  PlaceCandidate best;
  for (std::size_t i = 0; i < indexed_candidate_count; ++i) {
    ++last_stats_.full_descriptor_comparisons;
    const auto candidate_id = candidate_ids[i];
    const int coarse_shift =
        fastAlignUsingSectorKey(sector_keys_[query.id], sector_keys_[candidate_id]);
    const auto result = descriptorDistance(descriptors_[query.id],
                                           descriptors_[candidate_id],
                                           coarse_shift,
                                           last_stats_.full_shift_evaluations);
    if (result.distance < best.descriptor_distance) {
      best.keyframe_id = candidate_id;
      best.descriptor_distance = result.distance;
      int signed_shift = result.sector_shift;
      if (signed_shift > config_.num_sectors / 2) signed_shift -= config_.num_sectors;
      best.yaw_offset_rad = signed_shift * kTwoPi / config_.num_sectors;
    }
  }

  last_stats_.retrieval_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - retrieval_start).count();
  last_stats_.total_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
  if (best.descriptor_distance >= config_.distance_threshold) return std::nullopt;
  return best;
}

std::vector<PlaceCandidate> ScanContextLoopDetector::findCandidates(
    const pcl::PointCloud<SlamPoint> &query_cloud,
    const std::vector<Keyframe> &map_keyframes) {
  using Clock = std::chrono::steady_clock;
  const auto total_start = Clock::now();
  last_stats_ = {};
  last_stats_.history_size = map_keyframes.size();
  if (map_keyframes.empty()) return {};

  const auto descriptor_start = Clock::now();
  appendMissingDescriptors(map_keyframes);
  const auto query_descriptor = makeDescriptor(query_cloud);
  const auto query_ring_key = makeRingKey(query_descriptor);
  const auto query_sector_key = makeSectorKey(query_descriptor);
  last_stats_.descriptor_build_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - descriptor_start).count();

  std::vector<std::size_t> eligible_ids;
  eligible_ids.reserve(map_keyframes.size());
  for (std::size_t id = 0; id < map_keyframes.size(); ++id) eligible_ids.push_back(id);
  if (!ring_key_index_) {
    ring_key_index_ = std::make_unique<RingKeySearchIndex>(
        ring_keys_, eligible_ids, config_.num_rings);
  } else {
    ring_key_index_->synchronize(eligible_ids);
  }

  const auto retrieval_start = Clock::now();
  const std::size_t count = std::min(
      ring_key_index_->size(), static_cast<std::size_t>(config_.num_candidates));
  const auto candidate_ids = ring_key_index_->nearest(query_ring_key, count);
  std::vector<PlaceCandidate> accepted;
  accepted.reserve(candidate_ids.size());
  for (const auto candidate_id : candidate_ids) {
    ++last_stats_.full_descriptor_comparisons;
    const int coarse_shift = fastAlignUsingSectorKey(query_sector_key, sector_keys_[candidate_id]);
    const auto result = descriptorDistance(query_descriptor, descriptors_[candidate_id], coarse_shift,
                                           last_stats_.full_shift_evaluations);
    if (result.distance >= config_.distance_threshold) continue;
    int signed_shift = result.sector_shift;
    if (signed_shift > config_.num_sectors / 2) signed_shift -= config_.num_sectors;
    accepted.push_back({candidate_id, result.distance,
                        signed_shift * kTwoPi / config_.num_sectors});
  }
  std::sort(accepted.begin(), accepted.end(), [](const auto &a, const auto &b) {
    return a.descriptor_distance < b.descriptor_distance;
  });
  last_stats_.eligible_candidates = eligible_ids.size();
  last_stats_.indexed_candidates = ring_key_index_->size();
  last_stats_.shortlisted_candidates = candidate_ids.size();
  last_stats_.retrieval_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - retrieval_start).count();
  last_stats_.total_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
  return accepted;
}

}  // namespace kiss_matcher_slam
