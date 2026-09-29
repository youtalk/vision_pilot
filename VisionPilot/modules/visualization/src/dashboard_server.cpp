#include <visualization/dashboard_messages.hpp>
#include <visualization/dashboard_server.hpp>

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace visualization
{

namespace net = boost::asio;
namespace websocket = boost::beast::websocket;
using tcp = net::ip::tcp;

// Shared by the server object, the accept thread and every client thread, so a detached client
// thread never outlives the data it reads.
struct DashboardState
{
  std::string device;
  net::io_context ioc{1};
  tcp::acceptor acceptor{ioc};
  std::atomic<bool> running{true};
  std::mutex mutex;
  std::condition_variable cv;
  uint64_t seq = 0;
  std::vector<std::string> frame;
};

namespace
{

void serve_client(std::shared_ptr<DashboardState> s, tcp::socket socket)
{
  try {
    websocket::stream<tcp::socket> ws(std::move(socket));
    ws.accept();
    ws.text(true);
    ws.write(net::buffer(dashboard::hello(now_ms(), s->device)));
    uint64_t sent = 0;  // the current frame, if one exists, goes out right after hello
    while (s->running) {
      std::vector<std::string> frame;
      {
        std::unique_lock<std::mutex> lock(s->mutex);
        s->cv.wait(lock, [&] { return !s->running || s->seq != sent; });
        if (!s->running) break;
        sent = s->seq;  // a slow client jumps to the newest frame
        frame = s->frame;
      }
      for (const auto & m : frame) ws.write(net::buffer(m));
    }
  } catch (const std::exception &) {
    // The page closed or reloaded. The thread ends and nothing else needs cleanup.
  }
}

void accept_loop(std::shared_ptr<DashboardState> s)
{
  s->acceptor.non_blocking(true);
  while (s->running) {
    tcp::socket socket(s->ioc);
    boost::system::error_code ec;
    s->acceptor.accept(socket, ec);
    if (ec == net::error::would_block) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }
    if (ec) continue;
    std::thread(serve_client, s, std::move(socket)).detach();
  }
}

}  // namespace

DashboardServer::DashboardServer(uint16_t port, std::string device)
: state_(std::make_shared<DashboardState>())
{
  state_->device = std::move(device);
  const tcp::endpoint ep(tcp::v4(), port);
  state_->acceptor.open(ep.protocol());
  state_->acceptor.set_option(net::socket_base::reuse_address(true));
  state_->acceptor.bind(ep);
  state_->acceptor.listen();
  accept_thread_ = std::thread(accept_loop, state_);
  std::fprintf(stderr, "[Dashboard] stack endpoint on ws://0.0.0.0:%u/\n", port);
}

DashboardServer::~DashboardServer()
{
  state_->running = false;
  state_->cv.notify_all();
  if (accept_thread_.joinable()) accept_thread_.join();
}

void DashboardServer::publish(std::vector<std::string> frame)
{
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->frame = std::move(frame);
    ++state_->seq;
  }
  state_->cv.notify_all();
}

}  // namespace visualization
