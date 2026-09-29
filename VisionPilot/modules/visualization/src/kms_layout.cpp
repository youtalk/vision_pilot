#include <visualization/kms_layout.hpp>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <vector>

namespace visualization
{

KmsRect fit_centered(int screen_w, int screen_h, int frame_w, int frame_h)
{
  if (screen_w <= 0 || screen_h <= 0 || frame_w <= 0 || frame_h <= 0) return {};
  long long w = screen_w;
  long long h = w * frame_h / frame_w;
  if (h > screen_h) {
    h = screen_h;
    w = h * frame_w / frame_h;
  }
  const int ew = static_cast<int>(w) & ~1;
  const int eh = static_cast<int>(h) & ~1;
  return {(screen_w - ew) / 2, (screen_h - eh) / 2, ew, eh};
}

bool read_preferred_mode(const std::string & drm_dir, int & w, int & h)
{
  namespace fs = std::filesystem;
  std::error_code ec;
  std::vector<fs::path> connectors;
  for (const auto & e : fs::directory_iterator(drm_dir, ec)) {
    const std::string n = e.path().filename().string();
    if (n.rfind("card", 0) == 0 && n.find("-DP-") != std::string::npos)
      connectors.push_back(e.path());
  }
  std::sort(connectors.begin(), connectors.end());
  for (const auto & c : connectors) {
    std::ifstream status(c / "status");
    std::string s;
    if (!(status >> s) || s != "connected") continue;
    std::ifstream modes(c / "modes");
    std::string line;
    int mw = 0, mh = 0;
    if (
      !std::getline(modes, line) || std::sscanf(line.c_str(), "%dx%d", &mw, &mh) != 2 || mw <= 0 ||
      mh <= 0) {
      return false;
    }
    w = mw;
    h = mh;
    return true;
  }
  return false;
}

}  // namespace visualization
