#!/usr/bin/env bash
# Lighting A/B for the CES 2027 rig on rog-amd. Four cases, three server starts:
#   low-nosun-ae0   today's server defaults with no weather call (the workstation condition)
#   low-sun70-ae0   the 2026-09-16 D4 condition (70 degree sun added at run time)
#   low-nosun-ae1   engine auto exposure on, no sun
#   medium-nosun-ae0  quality Medium, no sun (may exceed the 8 GB GPU; a SIGKILL is a result)
#   lum_ab.sh <package-dir>
# One LUMINANCE case=<name> ... line per case. Pass band: mean 166-196 (workstation).
set -uo pipefail
PKG="${1:?package dir}"; here=$(cd "$(dirname "$0")" && pwd)
OUT="${LUM_OUT:-/tmp/lum-ab}"; mkdir -p "$OUT"
IMG="${VP_IMAGE:-visionpilot:si}"
export CARLA_BENCH_IF="${CARLA_BENCH_IF:-enx00e04c680c75}"
wpid=
spid=

stop_all() {
  [ -n "${wpid:-}" ] && kill "$wpid" 2>/dev/null
  [ -n "${spid:-}" ] && kill -- -"$spid" 2>/dev/null
  pkill -9 CarlaUnreal-Lin 2>/dev/null
  wpid=; spid=; sleep 3
}
trap stop_all EXIT

# probe <case> : the ego rig is already walking; sample 50 frames on domain 1.
# The sed tags LUMINANCE_FAIL too: it has no space after LUMINANCE, so the old
# pattern left a failed probe's line untagged and ambiguous across four cases.
probe() {
  docker run --rm --net=host --ipc=host -e ROS_DOMAIN_ID=1 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e CYCLONEDDS_URI=file:///ws/si/cyclonedds-bench.xml -e CARLA_BENCH_IF="$CARLA_BENCH_IF" \
    -v "$here:/ws/si:ro" -v "$OUT:/out" "$IMG" \
    "source /ws/install/setup.bash && python3 /ws/si/luminance_probe.py --frames 50 --ppm /out/$1.ppm" \
    | sed -E "s/^(LUMINANCE(_FAIL)?) /\\1 case=$1 /" | tee -a "$OUT/results.txt"
}

# walk <sun-or-empty> : start lane_walk.py with or without the sun. It spawns
# the rig; the camera is published by the server on domain 1.
walk() {
  CARLA_SUN_ALTITUDE="$1" ~/carla-venv/bin/python "$here/lane_walk.py" 400 8.0 > "$OUT/walk-$2.log" 2>&1 &
  wpid=$!
  for _ in $(seq 1 60); do n=$(grep -c '^WALK ego=' "$OUT/walk-$2.log" || true); [ "${n:-0}" -gt 0 ] && return 0; sleep 1; done
  echo "LUMINANCE_FAIL case=$2 reason=walk" | tee -a "$OUT/results.txt"; return 1
}

# server <quality> <autoexposure> <case> : each case keeps its own server log
# (CARLA_LOG), since the default /tmp/carla-server.log would otherwise be
# overwritten by the next case's server before anyone reads it.
server() {
  out=$(CARLA_QUALITY="$1" CARLA_AUTOEXPOSURE="$2" CARLA_LOG="$OUT/carla-$3.log" bash "$here/run-carla-server.sh" "$PKG") || { echo "$out"; echo "LUMINANCE_FAIL case=$3 reason=server" | tee -a "$OUT/results.txt"; return 1; }
  spid=$(sed -n 's/^CARLA_SERVER_UP pid=//p' <<<"$out")
}

: > "$OUT/results.txt"
# Server A: Low, auto exposure off. nosun first, then the sun: the weather
# persists in the world, so the order inside one server matters. The sun
# case runs only if this server came up. Without that check, a failed
# server here still let the sun case run, wait out lane_walk.py's CARLA
# timeout, and print reason=walk, a marker that names the wrong cause.
server Low 0 low-nosun-ae0
server_a_up=$?
if [ "$server_a_up" -eq 0 ]; then
  walk "" low-nosun-ae0 && probe low-nosun-ae0
  # Same settle as stop_all. Actor destruction is asynchronous, so without it
  # the second walk hits the still-present hero on spawn point 5 and burns 60 s
  # to report reason=walk.
  [ -n "${wpid:-}" ] && kill "$wpid" 2>/dev/null; wpid=; sleep 3
  walk 70 low-sun70-ae0 && probe low-sun70-ae0
fi
stop_all
server Low 1 low-nosun-ae1 && walk "" low-nosun-ae1 && probe low-nosun-ae1
stop_all
server Medium 0 medium-nosun-ae0 && walk "" medium-nosun-ae0 && probe medium-nosun-ae0
stop_all
cat "$OUT/results.txt"
