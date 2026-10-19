#include <visualization/object_tracker.hpp>

#include <algorithm>
#include <cmath>
#include <tuple>
#include <vector>

namespace visualization
{

std::vector<int> ObjectTracker::update(const std::vector<Point> & pts)
{
  // Greedy matching, closest pairs first.
  // ponytail: all track-point pairs, fine for the tens of boxes AutoSpeed returns per frame.
  std::vector<std::tuple<float, size_t, size_t>> pairs;
  for (size_t t = 0; t < tracks_.size(); ++t) {
    for (size_t p = 0; p < pts.size(); ++p) {
      const float d = std::hypot(tracks_[t].x - pts[p].x, tracks_[t].y - pts[p].y);
      if (d <= kGateM) pairs.emplace_back(d, t, p);
    }
  }
  std::sort(pairs.begin(), pairs.end());
  std::vector<int> ids(pts.size(), -1);
  std::vector<bool> matched(tracks_.size(), false);
  for (const auto & pair : pairs) {
    const size_t t = std::get<1>(pair);
    const size_t p = std::get<2>(pair);
    if (matched[t] || ids[p] >= 0) continue;
    matched[t] = true;
    ids[p] = tracks_[t].id;
    tracks_[t] = {tracks_[t].id, pts[p].x, pts[p].y, 0};
  }
  for (size_t t = 0; t < tracks_.size(); ++t) {
    if (!matched[t]) ++tracks_[t].missed;
  }
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(), [](const Track & k) { return k.missed > kMaxMissed; }),
    tracks_.end());
  for (size_t p = 0; p < pts.size(); ++p) {
    if (ids[p] >= 0) continue;
    ids[p] = next_id_++;
    tracks_.push_back({ids[p], pts[p].x, pts[p].y, 0});
  }
  return ids;
}

}  // namespace visualization
