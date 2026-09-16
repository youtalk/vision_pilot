#!/usr/bin/env bash
# Start the packaged CARLA server for the CES 2027 demo on rog-amd.
#   run-carla-server.sh <package-dir>
# Prints the package's fork SHA first (a package silently misses fixes made
# after it was cooked), starts Town04_Opt headless with native ROS 2 over
# CycloneDDS on domain 1, and waits for the RPC port.
# Markers: CARLA_SERVER sha=<sha> | CARLA_SERVER_UP pid=<pid> | CARLA_SERVER_FAIL reason=<slug>
set -uo pipefail
PKG="${1:-}"; [ -n "$PKG" ] || { echo "CARLA_SERVER_FAIL reason=bad_args"; exit 1; }
[ -f "$PKG/ces2027-package-sha.txt" ] || { echo "CARLA_SERVER_FAIL reason=no_sha_stamp"; exit 1; }
[ -x "$PKG/CarlaUnreal.sh" ] || { echo "CARLA_SERVER_FAIL reason=no_launcher"; exit 1; }

# CarlaUnreal.sh is a launcher, not necessarily the engine binary: if it forks
# rather than execs, "kill $pid" leaves the engine alive holding port 2000.
# setsid gives the server its own process group (pgid == pid, see below), so
# this kills the whole group, confirms death, and escalates if it lingers.
stop_server() {
  kill -- -"$1" 2>/dev/null
  for _ in $(seq 1 20); do kill -0 "$1" 2>/dev/null || return 0; sleep 0.5; done
  kill -9 -- -"$1" 2>/dev/null
}

here=$(cd "$(dirname "$0")" && pwd)
export CARLA_BENCH_IF="${CARLA_BENCH_IF:-enx00e04c680c75}"
export CYCLONEDDS_URI="file://$here/cyclonedds-bench.xml"
export ROS_DOMAIN_ID=1
MAP="${CARLA_MAP:-/Game/Carla/Maps/Town04_Opt}"
LOG="${CARLA_LOG:-/tmp/carla-server.log}"
echo "CARLA_SERVER sha=$(cat "$PKG/ces2027-package-sha.txt")"
if (exec 3<>/dev/tcp/127.0.0.1/2000) 2>/dev/null; then echo "CARLA_SERVER_FAIL reason=port_2000_busy"; exit 1; fi
setsid "$PKG/CarlaUnreal.sh" "$MAP" -RenderOffScreen -nosound --ros2 --rmw=cyclonedds --ros-domain-id=1 \
  -ExecCmds="r.DefaultFeature.AutoExposure 0" > "$LOG" 2>&1 &
pid=$!
for _ in $(seq 1 120); do
  if (exec 3<>/dev/tcp/127.0.0.1/2000) 2>/dev/null; then
    # The log is block-buffered (stdout redirected to a file), so the ROS2
    # line can be written but not yet flushed the instant port 2000 opens.
    # Poll for up to 10s instead of checking once, so a slow flush is not
    # mistaken for a real absence.
    ros2_ok=
    for _ in $(seq 1 10); do
      grep -q "ROS2: enabled with middleware 'cyclonedds'" "$LOG" 2>/dev/null && { ros2_ok=1; break; }
      sleep 1
    done
    [ -n "$ros2_ok" ] || { stop_server "$pid"; echo "CARLA_SERVER_FAIL reason=ros2_not_enabled"; exit 1; }
    echo "CARLA_SERVER_UP pid=$pid"; exit 0
  fi
  kill -0 "$pid" 2>/dev/null || { echo "CARLA_SERVER_FAIL reason=server_exited log=$LOG"; exit 1; }
  sleep 2
done
stop_server "$pid"; echo "CARLA_SERVER_FAIL reason=port_timeout"; exit 1
