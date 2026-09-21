#!/usr/bin/env bash
# Start the packaged CARLA server for the CES 2027 demo on rog-amd.
#   run-carla-server.sh <package-dir>
# Prints the package's fork SHA first (a package silently misses fixes made
# after it was cooked), starts Town04_Opt headless with native ROS 2 over
# CycloneDDS on domain 1, and waits for the RPC port.
# Markers: CARLA_SERVER sha=<sha> | CARLA_SERVER_UP pid=<pid> | CARLA_SERVER_FAIL reason=<slug>
set -uo pipefail
PKG="${1:-}"; [ -n "$PKG" ] || { echo "CARLA_SERVER_FAIL reason=bad_args"; exit 1; }
# The sha stamp sits at the package root, but the launcher sits one level
# down under Linux/. Do not "fix" one path to match the other.
[ -f "$PKG/ces2027-package-sha.txt" ] || { echo "CARLA_SERVER_FAIL reason=no_sha_stamp"; exit 1; }
[ -x "$PKG/Linux/CarlaUnreal.sh" ] || { echo "CARLA_SERVER_FAIL reason=no_launcher"; exit 1; }

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
# CycloneDDS binds 7400 + 250 * domain for SPDP multicast discovery. The server
# opens it only when ROS 2 really started, so it is the go/no-go signal below.
DDS_PORT=$((7400 + 250 * ROS_DOMAIN_ID))
ros2_port_bound() { ss -unap 2>/dev/null | grep ":$1 " | grep -q CarlaUnreal; }
MAP="${CARLA_MAP:-/Game/Carla/Maps/Town04_Opt}"
# rog-amd has an 8 GB RTX 4070 Laptop GPU. At the default quality Town04_Opt sits
# at the edge of that, and startup dies by SIGKILL with nothing in the log. Low
# peaks at about 4.9 GB and starts reliably. Raise it only on a bigger GPU.
QUALITY="${CARLA_QUALITY:-Low}"
# Auto exposure is off by default (the value the 2026-09-16 runs used). The
# lighting A/B sets CARLA_AUTOEXPOSURE=1 to see whether the engine's own
# metering removes the blown-out frame that a raised sun produced.
LOG="${CARLA_LOG:-/tmp/carla-server.log}"
echo "CARLA_SERVER sha=$(cat "$PKG/ces2027-package-sha.txt")"
if (exec 3<>/dev/tcp/127.0.0.1/2000) 2>/dev/null; then echo "CARLA_SERVER_FAIL reason=port_2000_busy"; exit 1; fi
setsid "$PKG/Linux/CarlaUnreal.sh" "$MAP" -RenderOffScreen -nosound -quality-level="$QUALITY" \
  --ros2 --rmw=cyclonedds --ros-domain-id=1 \
  -ExecCmds="r.DefaultFeature.AutoExposure ${CARLA_AUTOEXPOSURE:-0}" > "$LOG" 2>&1 &
pid=$!
for _ in $(seq 1 120); do
  if (exec 3<>/dev/tcp/127.0.0.1/2000) 2>/dev/null; then
    # Do NOT grep the log for "ROS2: enabled with middleware ...". CarlaEngine.cpp
    # emits that at UE_LOG Log verbosity, which a Shipping build strips, so the
    # packaged server can never print it and that check can never pass.
    # The bound SPDP port is a runtime fact that survives the strip.
    ros2_ok=
    for _ in $(seq 1 10); do
      ros2_port_bound "$DDS_PORT" && { ros2_ok=1; break; }
      sleep 1
    done
    [ -n "$ros2_ok" ] || { stop_server "$pid"; echo "CARLA_SERVER_FAIL reason=ros2_not_enabled"; exit 1; }
    echo "CARLA_SERVER_UP pid=$pid"; exit 0
  fi
  kill -0 "$pid" 2>/dev/null || { echo "CARLA_SERVER_FAIL reason=server_exited log=$LOG"; exit 1; }
  sleep 2
done
stop_server "$pid"; echo "CARLA_SERVER_FAIL reason=port_timeout"; exit 1
