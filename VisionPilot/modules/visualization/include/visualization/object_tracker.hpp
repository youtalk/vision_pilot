#pragma once

#include <vector>

namespace visualization
{

// Gives AutoSpeed boxes a stable id from frame to frame, by nearest neighbor in the ego frame.
class ObjectTracker
{
public:
  struct Point
  {
    float x, y;
  };

  // Returns one id per point, in the same order.
  std::vector<int> update(const std::vector<Point> & pts);

private:
  struct Track
  {
    int id;
    float x, y;
    int missed;
  };
  static constexpr float kGateM = 3.f;
  static constexpr int kMaxMissed = 5;
  std::vector<Track> tracks_;
  int next_id_ = 0;
};

}  // namespace visualization
