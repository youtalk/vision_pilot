#!/usr/bin/env bash
# Re-take the lane-detection baseline for the CARLA rig on the bench host.
#   lane_baseline.sh <package-dir> [measure-seconds]
# The ego drives on CARLA's own autopilot, not on VisionPilot's steering, so the
# result measures lane detection alone. A render change (for example the
# -quality-level the 8 GB bench GPU forces) moves this number, and a thin render
# then looks exactly like a planner fault in gate D4.
# Markers: LANE_BASELINE ... | LANE_BASELINE_FAIL reason=<slug>
set -uo pipefail
PKG="${1:-}"; SECS="${2:-120}"
here=$(cd "$(dirname "$0")" && pwd)
[ -n "$PKG" ] || { echo "LANE_BASELINE_FAIL reason=bad_args"; exit 1; }
IMG="${VP_IMAGE:-visionpilot:si}"
VPLOG="${VP_LOG:-/tmp/vp-lane.log}"
CACHE="${VP_CACHE:-/tmp/vp-trt-cache}"
mkdir -p "$CACHE"; rm -f "$VPLOG"

cleanup() {
  [ -n "${wpid:-}" ] && kill "$wpid" 2>/dev/null
  docker rm -f lb-bridge lb-vp > /dev/null 2>&1
  [ -n "${spid:-}" ] && kill -- -"$spid" 2>/dev/null
  pkill -9 CarlaUnreal-Lin 2>/dev/null
}
trap cleanup EXIT

out=$(bash "$here/run-carla-server.sh" "$PKG") || { echo "$out"; echo "LANE_BASELINE_FAIL reason=server"; exit 1; }
echo "$out"
spid=$(sed -n 's/^CARLA_SERVER_UP pid=//p' <<<"$out")

# No bridge and no traffic manager: lane_walk.py spawns the rig itself and
# teleports the ego along the lane centre, so the measurement is of perception
# alone. See its docstring.
# QT_QPA_PLATFORM=offscreen: VisionPilot initialises Qt even with
# visualization_on = false, and on a headless bench host the xcb plugin aborts
# the process after the models have already loaded.
docker run -d --gpus all --net=host --name lb-vp -e QT_QPA_PLATFORM=offscreen \
  -v "$CACHE:/cache" "$IMG" \
  'set -e
   cp /ws/config/visionpilot.carla.conf      /usr/share/visionpilot/config/vision_pilot.conf
   cp /ws/config/visionpilot_ros2.carla.conf /usr/share/visionpilot/config/vision_pilot_ros2.conf
   cp /ws/config/H_carla.yaml                /usr/share/visionpilot/config/H.yaml
   cp /ws/config/homography_C_matrix.yaml    /usr/share/visionpilot/config/homography_C_matrix.yaml
   source /ws/install/setup.bash
   exec /usr/bin/VisionPilot' > /dev/null || { echo "LANE_BASELINE_FAIL reason=vp"; exit 1; }

# The walk must start FIRST: VisionPilot cannot cycle until the camera exists, so
# waiting for its first [Lateral] line before spawning the rig deadlocks. The walk
# is given the measurement window plus a generous allowance for the TensorRT
# engine build, and is stopped as soon as the window closes.
~/carla-venv/bin/python "$here/lane_walk.py" "$((SECS + 900))" "${WALK_SPEED:-8.0}" > /tmp/lane-walk.log 2>&1 &
wpid=$!
ready=
for _ in $(seq 1 450); do
  # grep -c, not grep -q: under `set -o pipefail` a -q match closes the pipe,
  # docker logs takes SIGPIPE, and the pipeline reports failure, so the loop
  # never breaks once the log is big enough for the match to come early.
  seen=$(docker logs lb-vp 2>&1 | grep -c "\[Lateral\]" || true)
  [ "${seen:-0}" -gt 0 ] && { ready=1; break; }
  [ -n "$(docker ps -q --filter name=lb-vp)" ] || { docker logs lb-vp 2>&1 | tail -20; echo "LANE_BASELINE_FAIL reason=vp_exited"; exit 1; }
  kill -0 "$wpid" 2>/dev/null || { tail -5 /tmp/lane-walk.log; echo "LANE_BASELINE_FAIL reason=walk"; exit 1; }
  sleep 2
done
[ -n "$ready" ] || { echo "LANE_BASELINE_FAIL reason=vp_never_cycled"; exit 1; }
# Only cycles from here on count, so remember where the log had got to.
marker=$(docker logs lb-vp 2>&1 | wc -l)
sleep "$SECS"
# Only the cycles from the walk count, so drop everything logged before it began.
docker logs lb-vp 2>&1 | tail -n +"$marker" > "$VPLOG"
python3 "$here/lane_stats.py" "$VPLOG"
