#!/usr/bin/env bash
# Record one kill-route run as the five streams the demo video is composed from,
# and leave behind one self-describing run directory.
#
#   record-demo.sh <package-dir> [--run-dir <dir>] [--capture <file>] [--device <dev>]
#
# This is NOT a gate. It prints no gate marker of its own, and it passes
# run-d6.sh's own output, including its SI_STOP_PASS or SI_STOP_FAIL verdict,
# straight through.
#
# The five streams and who writes them:
#   chase/            demo_cam.py here           a chase view of the ego
#   camera/           record_camera.py here      what the board received
#   hud/              a pull script in openadkit  VisionPilot's HUD PNGs and its
#                     journal. This script only creates the directory and names
#                     it in the manifest; the pull script fills it afterwards.
#   cr52-console.txt  stamp_console.py here      the CR52 firmware console
#   trace.csv         si_stop_gate.py, via run-d6.sh, copied in at the end
#
# Options, each with an environment default:
#   --run-dir   DEMO_RUN_DIR   /tmp/demo-reel-<run id>
#   --capture   CR52_CAPTURE   /tmp/<device basename>.log, the tio capture that
#                              is ALREADY running on the console tty
#   --device    CR52_DEV       /dev/x5h2-cr52 (board 2); names the default capture
# Also honoured, as run-d6.sh and lum_ab.sh honour them: DRIVE_S, CARLA_BENCH_IF,
# VP_IMAGE, CARLA_PYTHON, plus DEMO_GROW_S (default 5) for the console check.
#
# jpeg_bridge.py must already be running on this host: record_camera.py records
# its output topic and nothing here starts it.
#
# Markers: DEMO_REC_READY once every recorder is up, then
# DEMO_REC_DONE streams=5 dir=<run dir> or DEMO_REC_FAIL reason=<slug> dir=<dir>.
set -uo pipefail
here=$(cd "$(dirname "$0")" && pwd)
PKG="${1:?package dir}"; shift
RUN_ID=$(date +%Y%m%d-%H%M%S)
CR52_DEV="${CR52_DEV:-/dev/x5h2-cr52}"
CAPTURE="${CR52_CAPTURE:-}"
RUN="${DEMO_RUN_DIR:-}"
while [ $# -gt 0 ]; do
  case "$1" in
    --run-dir) RUN="${2:?run dir}"; shift 2 ;;
    --capture) CAPTURE="${2:?capture file}"; shift 2 ;;
    --device) CR52_DEV="${2:?device}"; shift 2 ;;
    *) echo "DEMO_REC_FAIL reason=bad_args"; exit 1 ;;
  esac
done
RUN="${RUN:-/tmp/demo-reel-$RUN_ID}"
# The capture defaults to a name derived from the device, because the two are
# always set as a pair on this bench: passing only one of them is how a run ends
# up stamping board 1's console onto board 2's video.
CAPTURE="${CAPTURE:-/tmp/$(basename "$CR52_DEV").log}"
GROW_S="${DEMO_GROW_S:-5}"
DRIVE_S="${DRIVE_S:-40}"
IMG="${VP_IMAGE:-visionpilot:si}"
CARLA_PYTHON="${CARLA_PYTHON:-$HOME/carla-venv/bin/python}"
# Both recorders must outlive the gate: run-d6.sh fires the fault DRIVE_S after
# the bridge is up and si_stop_gate.py then measures for 30 s. They are stopped
# by signal as soon as run-d6.sh returns, so this is only a backstop for a
# driver that dies before it gets there.
REC_S=$((DRIVE_S + 90))

mkdir -p "$RUN/chase" "$RUN/camera" "$RUN/hud" || { echo "DEMO_REC_FAIL reason=bad_run_dir"; exit 1; }
fail() { echo "DEMO_REC_FAIL reason=$1 dir=$RUN"; exit 1; }
echo "record-demo run_id=$RUN_ID dir=$RUN device=$CR52_DEV capture=$CAPTURE"

stamp_pid=
cam_pid=
# shellcheck disable=SC2329  # runs from the EXIT trap below
cleanup() {
  [ -n "${stamp_pid:-}" ] && kill "$stamp_pid" 2>/dev/null
  [ -n "${cam_pid:-}" ] && kill "$cam_pid" 2>/dev/null
  docker rm -f demo-rec-cam > /dev/null 2>&1
}
trap cleanup EXIT

# Before anything else starts. A tio capture that died and a CR52 that halted
# look exactly alike in a file that is not growing, and reading one as the other
# has already cost this bench a wrong diagnosis. The firmware prints a periodic
# line, so a live capture always grows within a few seconds; raise DEMO_GROW_S if
# that print period is ever made longer than the default 5 s.
[ -r "$CAPTURE" ] || fail console_missing
before=$(wc -c < "$CAPTURE")
sleep "$GROW_S"
after=$(wc -c < "$CAPTURE")
[ "$after" -gt "$before" ] || fail console_not_growing

python3 "$here/stamp_console.py" "$CAPTURE" > "$RUN/cr52-console.txt" 2> "$RUN/stamp.log" &
stamp_pid=$!
# grep -c on a file, never a pipe into grep -q: grep -q exits at the first match
# and the writer then takes SIGPIPE, which under `set -o pipefail` turns a
# healthy check into a 141 and a random false failure.
for _ in $(seq 1 20); do
  n=$(grep -c '^STAMP ready' "$RUN/stamp.log" 2>/dev/null || true)
  [ "${n:-0}" -gt 0 ] && break
  sleep 1
done
[ "${n:-0}" -gt 0 ] || fail stamper

# run-d6.sh owns the simulator: it starts the CARLA server AND the bridge that
# spawns the hero, so neither CARLA-side recorder can attach before it runs. The
# recorders go up as soon as the server is, and well before FAULT_AT, which
# run-d6.sh does not fix until the bridge has come up after CARLA_SERVER_UP.
( bash "$here/run-d6.sh" "$PKG" kill 2>&1 | tee "$RUN/run-d6.txt" ) &
d6_pid=$!
for _ in $(seq 1 180); do
  n=$(grep -c '^CARLA_SERVER_UP' "$RUN/run-d6.txt" 2>/dev/null || true)
  [ "${n:-0}" -gt 0 ] && break
  sleep 1
done
[ "${n:-0}" -gt 0 ] || fail server

# rclpy lives in the image, not on the host, so the compressed-topic recorder
# runs in a container on the same DDS configuration as every other si node.
# `exec python3` so docker stop's SIGTERM reaches python instead of the shell.
docker run -d --name demo-rec-cam --net=host --ipc=host \
  -e ROS_DOMAIN_ID=1 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///ws/si/cyclonedds-bench.xml \
  -e CARLA_BENCH_IF="${CARLA_BENCH_IF:-enx00e04c680c75}" \
  -v "$here:/ws/si:ro" -v "$RUN/camera:/camera" "$IMG" \
  "source /ws/install/setup.bash && exec python3 /ws/si/record_camera.py /camera --seconds $REC_S" \
  > /dev/null || fail rec_cam
# The chase camera is a raw CARLA client, so it runs in the host carla venv
# (lum_ab.sh runs lane_walk.py the same way).
"$CARLA_PYTHON" "$here/demo_cam.py" "$RUN/chase" --seconds "$REC_S" > "$RUN/chase.log" 2>&1 &
cam_pid=$!
rec_ready=0
chase_ready=0
for _ in $(seq 1 120); do
  rec_ready=$(docker logs demo-rec-cam 2>&1 | grep -c '^REC_CAM ready' || true)
  chase_ready=$(grep -c '^DEMO_CAM ready' "$RUN/chase.log" 2>/dev/null || true)
  [ "${rec_ready:-0}" -gt 0 ] && [ "${chase_ready:-0}" -gt 0 ] && break
  sleep 1
done
[ "${rec_ready:-0}" -gt 0 ] || fail rec_cam_not_ready
[ "${chase_ready:-0}" -gt 0 ] || fail chase_not_ready
echo "DEMO_REC_READY"

# pipefail is what keeps `| tee` above from hiding run-d6.sh's exit status, but
# that status is deliberately not a verdict here: a gate FAIL such as
# stop_too_far is still a perfectly composable recording, and a recorder must
# not turn one into a recording failure. What a recording cannot survive is a
# missing fault instant, which is checked below.
wait "$d6_pid"

docker stop -t 20 demo-rec-cam > /dev/null 2>&1
docker logs demo-rec-cam > "$RUN/camera.log" 2>&1
kill "$cam_pid" 2>/dev/null; wait "$cam_pid" 2>/dev/null
kill "$stamp_pid" 2>/dev/null; wait "$stamp_pid" 2>/dev/null
cam_pid=
stamp_pid=

# FAULT_AT comes out of run-d6.sh and is never recomputed here. si_stop_gate.py
# already wrote trace.csv with times relative to that exact value, so a second
# opinion about the fault instant would slide the trace against every other
# stream by the difference.
fault_at=$(awk '/^SI_FAULT_INJECTED/ { for (i = 1; i <= NF; i++) if ($i ~ /^t=/) v = substr($i, 3) } END { print v }' "$RUN/run-d6.txt")
[ -n "$fault_at" ] || fail no_fault_at

d6log=$(sed -n 's/^D6 logs in //p' "$RUN/run-d6.txt" | tail -1)
[ -n "$d6log" ] || fail no_trace
[ -f "$d6log/snaps/trace.csv" ] || fail no_trace
cp "$d6log/snaps/trace.csv" "$RUN/trace.csv" || fail no_trace

# An index with nothing but its header is a pane the composer places and then
# shows nothing in. Catch that while the run is still in front of someone, and
# name which camera it was: the two have completely different causes (no hero
# versus no jpeg_bridge.py).
for stream in chase camera; do
  rows=$(wc -l < "$RUN/$stream/index.csv" 2>/dev/null || echo 0)
  [ "${rows:-0}" -gt 1 ] || fail "${stream}_empty"
done
[ -s "$RUN/cr52-console.txt" ] || fail console_empty

python3 "$here/demo_streams.py" --run-id "$RUN_ID" --mode kill --fault-at "$fault_at" \
  > "$RUN/manifest.json" || fail manifest
echo "DEMO_REC_DONE streams=5 dir=$RUN"
