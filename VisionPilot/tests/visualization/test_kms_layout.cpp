// Pure-logic checks for the KMS display sink: where the HUD lands on the
// screen, and which connector mode it is fitted to. No GStreamer, no DRM.
#include <visualization/kms_layout.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

namespace fs = std::filesystem;
using visualization::fit_centered;
using visualization::KmsRect;
using visualization::read_preferred_mode;

static int failures = 0;
#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      ++failures;                                                          \
    }                                                                      \
  } while (0)

static bool same(const KmsRect & r, int x, int y, int w, int h)
{
  return r.x == x && r.y == y && r.w == w && r.h == h;
}

static void connector(
  const fs::path & drm, const std::string & name, const std::string & status,
  const std::string & modes, const std::string & enabled = "enabled")
{
  fs::create_directories(drm / name);
  std::ofstream(drm / name / "status") << status << "\n";
  std::ofstream(drm / name / "enabled") << enabled << "\n";
  std::ofstream(drm / name / "modes") << modes;
}

int main()
{
  // A whole-number scale only: the 1024x512 HUD at 2x on the LG's 3440x1440
  // native mode, not the 2.8x that would fill its height. The scaler then
  // writes half the pixels, and that time is the HUD's frame cost.
  CHECK(same(fit_centered(3440, 1440, 1024, 512), 696, 208, 2048, 1024));
  // Exact fit.
  CHECK(same(fit_centered(1920, 1080, 1920, 1080), 0, 0, 1920, 1080));
  // 1.4x would fit, so 1x it is, centred.
  CHECK(same(fit_centered(1920, 1080, 1024, 768), 448, 156, 1024, 768));
  // Width-limited: letter-boxed.
  CHECK(same(fit_centered(1024, 1024, 2048, 1024), 0, 256, 1024, 512));
  // Odd sizes round down to even.
  CHECK(same(fit_centered(1921, 1081, 1921, 1081), 0, 0, 1920, 1080));
  // Degenerate input gives an empty rectangle.
  CHECK(same(fit_centered(0, 1080, 1920, 1080), 0, 0, 0, 0));
  CHECK(same(fit_centered(1920, 1080, 0, 0), 0, 0, 0, 0));

  const fs::path root = fs::temp_directory_path() / "test_kms_layout";
  fs::remove_all(root);
  int w = -1, h = -1;

  // No DRM directory at all.
  CHECK(!read_preferred_mode((root / "absent").string(), w, h));
  CHECK(w == -1 && h == -1);

  // Only card0 counts: kmssink driver-name=rcar-vcon opens the first card of
  // that driver, and all three X5H cards are rcar-vcon. A lit monitor on
  // another card would otherwise pass this check while kmssink drives card0.
  const fs::path a = root / "a";
  connector(a, "card0-DP-1", "disconnected", "");
  connector(a, "card0-Writeback-1", "unknown", "");
  connector(a, "card1-DP-2", "connected", "3440x1440\n1920x1080\n");
  CHECK(!read_preferred_mode(a.string(), w, h));

  // The connected DP connector of card0 wins; a writeback connector is skipped.
  const fs::path g = root / "g";
  connector(g, "card0-Writeback-1", "unknown", "");
  connector(g, "card0-DP-1", "connected", "3440x1440\n1920x1080\n");
  CHECK(read_preferred_mode(g.string(), w, h));
  CHECK(w == 3440 && h == 1440);

  // Connected, but the kernel lists no mode yet.
  const fs::path b = root / "b";
  connector(b, "card0-DP-1", "connected", "");
  w = h = -1;
  CHECK(!read_preferred_mode(b.string(), w, h));
  CHECK(w == -1 && h == -1);

  // Connected, with a line that is not a mode.
  const fs::path c = root / "c";
  connector(c, "card0-DP-1", "connected", "garbage\n");
  CHECK(!read_preferred_mode(c.string(), w, h));

  // Only DP counts.
  const fs::path d = root / "d";
  connector(d, "card0-HDMI-A-1", "connected", "1920x1080\n");
  CHECK(!read_preferred_mode(d.string(), w, h));

  // Connected, but no CRTC drives it yet (the kernel console has not lit
  // it). kmssink would then set a mode itself, which the sink must never do.
  const fs::path e = root / "e";
  connector(e, "card0-DP-1", "connected", "3440x1440\n", "disabled");
  CHECK(!read_preferred_mode(e.string(), w, h));

  // The same unlit card0 connector, with another card lit: still no. kmssink
  // would set a mode on card0, and the next restart would release it.
  const fs::path f = root / "f";
  connector(f, "card0-DP-1", "connected", "3440x1440\n", "disabled");
  connector(f, "card1-DP-2", "connected", "1920x1080\n");
  CHECK(!read_preferred_mode(f.string(), w, h));

  fs::remove_all(root);
  if (failures == 0) std::printf("TEST_PASS test_kms_layout\n");
  return failures == 0 ? 0 : 1;
}
