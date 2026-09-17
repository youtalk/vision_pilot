#!/usr/bin/env bash
# The sun is opt-in: no CARLA_SUN_ITUDE, no set_weather call. Both the bridge's
# config_carla.py and the perception walk must follow the same rule, and the
# old unconditional default of 70 degrees must be gone.
set -u
here=$(cd "$(dirname "$0")" && pwd)
fail() { echo "TEST_FAIL test-daylight reason=$1"; exit 1; }
cfg="$here/../src/carla_bridge_bringup/scripts/config_carla.py"; walk="$here/lane_walk.py"
for f in "$cfg" "$walk"; do
  python3 -m py_compile "$f" || fail "compile_$(basename "$f")"
  n=$(grep -c 'os.environ.get("CARLA_SUN_ALTITUDE")' "$f"); [ "$n" -ge 1 ] || fail "no_optin_$(basename "$f")"
  n=$(grep -c 'CARLA_SUN_ALTITUDE", "70.0"' "$f" || true); [ "$n" -eq 0 ] || fail "default_sun_$(basename "$f")"
done
echo "TEST_PASS test-daylight"
