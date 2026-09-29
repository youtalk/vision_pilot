#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace visualization
{

// The sender's monotonic clock in milliseconds, for `t_ms`.
inline double now_ms()
{
  const auto t = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration<double, std::milli>(t).count();
}

struct DashboardState;

// WebSocket server for the dashboard's `stack` role, on any request path. Every client gets
// `hello`, then each frame published after it connected. A slow client skips frames, so
// publish() never waits for a client.
class DashboardServer
{
public:
  // Throws when the port cannot be bound.
  DashboardServer(uint16_t port, std::string device);
  ~DashboardServer();
  DashboardServer(const DashboardServer &) = delete;
  DashboardServer & operator=(const DashboardServer &) = delete;

  // Replaces the current frame and wakes every client.
  void publish(std::vector<std::string> frame);

private:
  std::shared_ptr<DashboardState> state_;
  std::thread accept_thread_;
};

}  // namespace visualization
