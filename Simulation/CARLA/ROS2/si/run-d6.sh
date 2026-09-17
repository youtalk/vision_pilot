#!/usr/bin/env bash
# Gate D6 on rog-amd, three modes:
#   standin  bench rehearsal, no board: VisionPilot runs here, a stand-in
#            publishes the CR52 ramp (Task 7 of the phase 2 plan)
#   kill     full stack: VisionPilot on board 1; si_fault.sh kill (700 ms budget)
#   channel  full stack: si_fault.sh channel, fault=1 on rpmsg-si (200 ms budget)
#   run-d6.sh <package-dir> <standin|kill|channel>
# Markers: SI_FAULT_INJECTED ... then SI_STOP_PASS ... | SI_STOP_FAIL reason=<slug>
#
# Timing: FAULT_AT is fixed to "now + DRIVE_S" BEFORE d6-gate starts, and
# passed to it as --fault-at. That is not cosmetic: si_fault.sh sleeps until
# a FUTURE instant, so a FAULT_AT set to "now" fires the fault before d6-gate
# has even subscribed, and the gate misses the samples it exists to measure.
# Driving time and the gate's subscription warm-up now overlap on purpose.
set -uo pipefail
PKG="${1:?package dir}"; MODE="${2:?standin|kill|channel}"; here=$(cd "$(dirname "$0")" && pwd)
# shellcheck source=/dev/null
. "$here/route.env"
LOG=/tmp/d6-$(date +%Y%m%d-%H%M%S); mkdir -p "$LOG/snaps"
DRIVE_S="${DRIVE_S:-40}"      # seconds of driving before the fault
case "$MODE" in
  # standin has no CR52 firmware to bound: the rehearsal proves the arbiter
  # switches, the ego stops, and the snapshots are written, all without a
  # board. A container cold start (v0 sample, docker rm, cold docker run)
  # must not read as a firmware latency failure. The real 700 ms budget is
  # measured on hardware in a later task.
  standin) BUDGET="${STANDIN_BUDGET:-5000}" ;; kill) BUDGET=700 ;; channel) BUDGET=200 ;;
  *) echo "SI_STOP_FAIL reason=bad_args"; exit 1 ;;
esac

# shellcheck disable=SC2329  # runs from the EXIT trap below
cleanup() {
  docker rm -f d6-bridge d6-vp d6-snap d6-gate > /dev/null 2>&1
  [ -n "${sipid:-}" ] && kill "$sipid" 2>/dev/null
  [ -n "${spid:-}" ] && kill -- -"$spid" 2>/dev/null
  pkill -9 CarlaUnreal-Lin 2>/dev/null
}
trap cleanup EXIT
fail() { echo "SI_STOP_FAIL reason=$1 log=$LOG"; exit 1; }

out=$(bash "$here/run-carla-server.sh" "$PKG"); echo "$out" | tee "$LOG/server.txt"
grep -q '^CARLA_SERVER_UP' <<<"$out" || fail server
spid=$(sed -n 's/^CARLA_SERVER_UP pid=//p' <<<"$out")

CFG_IN_IMAGE=/ws/install/carla_bridge_bringup/share/carla_bridge_bringup/scripts/config_carla.py
DOCKER="docker run -d --net=host --ipc=host --gpus all -e ROS_DOMAIN_ID=1 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///ws/si/cyclonedds-bench.xml -e CARLA_BENCH_IF=${CARLA_BENCH_IF:-enx00e04c680c75} -e CARLA_SUN_ALTITUDE=${CARLA_SUN_ALTITUDE:-} -v $here:/ws/si:ro -v $LOG/snaps:/snaps -v $here/../src/carla_bridge_bringup/scripts/config_carla.py:$CFG_IN_IMAGE:ro"
$DOCKER --name d6-bridge -e SPAWN_INDEX="$SPAWN_INDEX" visionpilot:si \
  "source /ws/install/setup.bash && ros2 launch carla_bridge_bringup carla_bridge.launch.py host:=127.0.0.1 port:=2000 rig_file:=/ws/config/carla10.json" > /dev/null || fail bridge
for _ in $(seq 1 45); do
  n=$(docker logs d6-bridge 2>&1 | grep -c "Running\.\.\." || true)
  [ "${n:-0}" -gt 0 ] && break
  sleep 2
done

if [ "$MODE" = standin ]; then
  $DOCKER --name d6-vp -e QT_QPA_PLATFORM=offscreen \
    -v "$here/../config/visionpilot.carla.conf:/usr/share/visionpilot/config/vision_pilot.conf:ro" \
    -v "$here/../config/visionpilot_ros2.carla.conf:/usr/share/visionpilot/config/vision_pilot_ros2.conf:ro" \
    -v "$here/../config/H_carla.yaml:/usr/share/visionpilot/config/H.yaml:ro" \
    -v "$here/../config/homography_C_matrix.yaml:/usr/share/visionpilot/config/homography_C_matrix.yaml:ro" \
    visionpilot:si "source /ws/install/setup.bash && cd /usr/share/visionpilot && exec /usr/bin/VisionPilot --no-window" > /dev/null || fail vp
  for _ in $(seq 1 300); do
    n=$(docker logs d6-vp 2>&1 | grep -c "\[Lateral\]" || true)
    [ "${n:-0}" -gt 0 ] && break
    [ -n "$(docker ps -q --filter name=d6-vp)" ] || { docker logs d6-vp > "$LOG/vp.log" 2>&1; fail vp_exited; }
    sleep 2
  done
fi
# In kill/channel mode VisionPilot runs on the board; the operator confirmed
# X5H_DEMO_UP units=5 before calling this script.

# FAULT_AT is fixed NOW, before d6-gate starts, and handed to it as
# --fault-at so the gate can warm up its subscriptions before the fault
# fires. Do not add a bare `sleep "$DRIVE_S"` here: the driving time and the
# gate's warm-up are meant to overlap, not happen back to back.
FAULT_AT=$(awk -v n="$(date +%s.%N)" -v d="$DRIVE_S" 'BEGIN { printf "%.3f", n + d }')
$DOCKER --name d6-gate visionpilot:si "source /ws/install/setup.bash && python3 /ws/si/si_stop_gate.py --fault-at $FAULT_AT --window 30 --max-latency-ms $BUDGET" > /dev/null || fail gate
$DOCKER --name d6-snap visionpilot:si "source /ws/install/setup.bash && python3 /ws/si/snap.py /snaps --seconds $((DRIVE_S + 40))" > /dev/null || fail snap
case "$MODE" in
  standin)
    # v0 from the ego-state publisher's odometry, one sample, taken DURING
    # the drive window (not after FAULT_AT — the sample, the container
    # removal and the stand-in's cold start all have to fit before the fault).
    v0=$(docker run --rm --net=host --ipc=host -e ROS_DOMAIN_ID=1 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///ws/si/cyclonedds-bench.xml -e CARLA_BENCH_IF="${CARLA_BENCH_IF:-enx00e04c680c75}" -v "$here:/ws/si:ro" visionpilot:si \
      "source /ws/install/setup.bash && timeout 10 ros2 topic echo --once /localization/kinematic_state --field twist.twist.linear.x" | tr -d '\r' | tail -1)
    # Wait for FAULT_AT itself (si_fault.sh's own idiom) before "failing"
    # VisionPilot, so the fault instant means the same thing in every mode.
    sleep "$(awk -v a="$FAULT_AT" -v n="$(date +%s.%N)" 'BEGIN { d = a - n; print (d > 0) ? d : 0 }')"
    docker rm -f d6-vp > /dev/null 2>&1      # VisionPilot "fails"
    echo "SI_FAULT_INJECTED mode=standin t=$FAULT_AT v0=$v0"
    docker run --rm --net=host --ipc=host -e ROS_DOMAIN_ID=1 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///ws/si/cyclonedds-bench.xml -e CARLA_BENCH_IF="${CARLA_BENCH_IF:-enx00e04c680c75}" -v "$here:/ws/si:ro" visionpilot:si \
      "source /ws/install/setup.bash && python3 /ws/si/si_standin.py --v0 $v0" > "$LOG/standin.log" 2>&1 &
    sipid=$! ;;
  kill|channel)
    bash "$here/si_fault.sh" "$MODE" "$FAULT_AT" | tee "$LOG/fault.txt" ;;
esac
docker wait d6-gate > /dev/null; docker logs d6-gate 2>&1 | tee "$LOG/gate.txt"
docker wait d6-snap > /dev/null; docker logs d6-snap 2>&1 | tail -1
docker logs d6-bridge > "$LOG/bridge.log" 2>&1
echo "D6 logs in $LOG"
grep -q '^SI_STOP_PASS' "$LOG/gate.txt"
