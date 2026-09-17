#!/usr/bin/env bash
# Gate D4 (reworded 2026-09-17): one closed lap of the spawn-5 loop at 12 m/s, |cte| < 1.75 m.
#   run-d4.sh <package-dir>
# Everything ROS 2 runs in the visionpilot:si image on the host network with the
# bench CycloneDDS configuration; the lap gate runs in the host venv.
# Markers: D4_LAP_PASS ... | D4_LAP_FAIL reason=<slug>
set -uo pipefail
PKG="${1:?package dir}"; here=$(cd "$(dirname "$0")" && pwd)
# shellcheck source=/dev/null
. "$here/route.env"
VP_SECONDS="${VP_SECONDS:-900}"
LOG=/tmp/d4-$(date +%Y%m%d-%H%M%S); mkdir -p "$LOG"

# No --rm on the containers: if VisionPilot exits at once, --rm deletes it along
# with the log that says why, and the failure reads as "No such container".
# The CARLA server is stopped here too, otherwise it holds port 2000 and the
# next run refuses to start.
# shellcheck disable=SC2329  # runs from the EXIT trap below
cleanup() {
  docker rm -f d4-bridge d4-vp > /dev/null 2>&1
  [ -n "${spid:-}" ] && kill -- -"$spid" 2>/dev/null
  pkill -9 CarlaUnreal-Lin 2>/dev/null
}
trap cleanup EXIT
fail() { echo "D4_LAP_FAIL reason=$1 log=$LOG"; exit 1; }

out=$(bash "$here/run-carla-server.sh" "$PKG"); echo "$out" | tee "$LOG/server.txt"
grep -q '^CARLA_SERVER_UP' <<<"$out" || fail server
spid=$(sed -n 's/^CARLA_SERVER_UP pid=//p' <<<"$out")

CFG_IN_IMAGE=/ws/install/carla_bridge_bringup/share/carla_bridge_bringup/scripts/config_carla.py
# The image on rog-amd cannot be rebuilt there (no visionpilot:gpu-ros2 base),
# so the changed config_carla.py is mounted over its installed copy.
# ponytail: bind-mount override; rebuild and re-ship visionpilot:si before the booth.
DOCKER="docker run -d --net=host --ipc=host --gpus all -e ROS_DOMAIN_ID=1 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///ws/si/cyclonedds-bench.xml -e CARLA_BENCH_IF=${CARLA_BENCH_IF:-enx00e04c680c75} -e CARLA_SUN_ALTITUDE=${CARLA_SUN_ALTITUDE:-} -v $here/../src/carla_bridge_bringup/scripts/config_carla.py:$CFG_IN_IMAGE:ro"
# SPAWN_INDEX, not CARLA_SPAWN_INDEX: config_carla.py reads SPAWN_INDEX.
$DOCKER --name d4-bridge -e SPAWN_INDEX="$SPAWN_INDEX" visionpilot:si \
  "source /ws/install/setup.bash && ros2 launch carla_bridge_bringup carla_bridge.launch.py host:=127.0.0.1 port:=2000 rig_file:=/ws/config/carla10.json" > /dev/null || fail bridge
for _ in $(seq 1 45); do
  n=$(docker logs d4-bridge 2>&1 | grep -c "Running\.\.\." || true)
  [ "${n:-0}" -gt 0 ] && break
  sleep 2
done

# source setup.bash first: without it /opt/ros/jazzy/lib is off the library
# path and rmw_cyclonedds_cpp cannot load libddsc.so.0, so VisionPilot exits 1
# straight after the models load.
$DOCKER --name d4-vp -e QT_QPA_PLATFORM=offscreen \
  -v "$here/../config/visionpilot.carla.conf:/usr/share/visionpilot/config/vision_pilot.conf:ro" \
  -v "$here/../config/visionpilot_ros2.carla.conf:/usr/share/visionpilot/config/vision_pilot_ros2.conf:ro" \
  -v "$here/../config/H_carla.yaml:/usr/share/visionpilot/config/H.yaml:ro" \
  -v "$here/../config/homography_C_matrix.yaml:/usr/share/visionpilot/config/homography_C_matrix.yaml:ro" \
  visionpilot:si "source /ws/install/setup.bash && cd /usr/share/visionpilot && exec /usr/bin/VisionPilot --no-window" > /dev/null || fail vp
# grep -c, not grep -q: under `set -o pipefail` a -q match closes the pipe and
# docker logs takes SIGPIPE, so the pipeline reports failure and the loop never ends.
for _ in $(seq 1 300); do
  n=$(docker logs d4-vp 2>&1 | grep -c "\[Lateral\]" || true)
  [ "${n:-0}" -gt 0 ] && break
  [ -n "$(docker ps -q --filter name=d4-vp)" ] || { docker logs d4-vp > "$LOG/vp.log" 2>&1; fail vp_exited; }
  sleep 2
done

~/carla-venv/bin/python "$here/lap_gate.py" "$VP_SECONDS" \
  --max-cte "${MAX_CTE:-1.75}" --min-lap-m "$MIN_LAP_M" --csv "$LOG/lap.csv" | tee "$LOG/gate.txt"
rc=${PIPESTATUS[0]}
docker logs d4-vp > "$LOG/vp.log" 2>&1; docker logs d4-bridge > "$LOG/bridge.log" 2>&1
echo "D4 logs in $LOG"
exit "$rc"
