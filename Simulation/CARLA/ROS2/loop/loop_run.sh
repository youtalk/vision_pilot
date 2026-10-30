#!/usr/bin/env bash
# Closed-loop runner for the PoV_Highway scenarios on a CARLA 0.10
# (ue58-dev) Shipping package. Start the CARLA server first.
#
#   loop_run.sh up         build the images if missing, then create the
#                          pov-vp and pov-bridge containers
#   loop_run.sh run <tag>  one run: clear the world, restart VisionPilot,
#                          start the scenario, then the bridge (it must
#                          find the new hero), then the observer
#
# Environment:
#   PY               Python 3.12 with the CARLA 0.10 wheel (default python3)
#   LOG_DIR          run logs (default ./loop-logs)
#   CARLA_WHEEL_DIR  the cp312 wheel, unzipped (for `up`)
#   VP_IMAGE         default visionpilot:gpu-ros2-cyclone
#   BRIDGE_IMAGE     default visionpilot:si
# Extra variables (CUTOUT_TRIGGER_DIST, SCENARIO_FAST_START_M, ...) pass
# through to the scenario.
set -euo pipefail
ROS2=$(cd "$(dirname "$0")/.." && pwd)
PY=${PY:-python3}
VP_IMAGE=${VP_IMAGE:-visionpilot:gpu-ros2-cyclone}
BRIDGE_IMAGE=${BRIDGE_IMAGE:-visionpilot:si}
ROS_ENV=(-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e ROS_DOMAIN_ID=42)

up() {
  : "${CARLA_WHEEL_DIR:?set CARLA_WHEEL_DIR to the unzipped carla cp312 wheel}"
  # VisionPilot/docker/build.sh --gpu --ros2 makes visionpilot:gpu-ros2.
  # Fast DDS shows only /clock from the 0.10 server, so add CycloneDDS.
  docker image inspect "$VP_IMAGE" >/dev/null 2>&1 || docker build -t "$VP_IMAGE" - <<'EOF'
FROM visionpilot:gpu-ros2
RUN apt-get update && apt-get install -y --no-install-recommends ros-jazzy-rmw-cyclonedds-cpp && rm -rf /var/lib/apt/lists/*
EOF
  [ -f "$ROS2/install/setup.bash" ] || docker run --rm -v "$ROS2:/pov" -w /pov --entrypoint bash "$BRIDGE_IMAGE" \
    -c 'source /opt/ros/jazzy/setup.bash && colcon build'
  docker rm -f pov-vp pov-bridge >/dev/null 2>&1 || true
  docker create --name pov-vp --gpus all --network host --ipc host "${ROS_ENV[@]}" \
    -e HOME=/tmp -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix "$VP_IMAGE" >/dev/null
  docker create --name pov-bridge --network host --ipc host "${ROS_ENV[@]}" \
    -v "$ROS2:/pov" -v "$CARLA_WHEEL_DIR:/carla312:ro" --entrypoint bash "$BRIDGE_IMAGE" -c \
    'source /opt/ros/jazzy/setup.bash && source /pov/install/setup.bash && export PYTHONPATH=/carla312:$PYTHONPATH && python3 -c "import carla; print(carla.__file__)" && (ros2 run carla_vehicle_speed_publisher carla_vehicle_speed_publisher_node --ros-args -p use_sim_time:=true &) && exec ros2 run carla_control_publisher carla_control_publisher_node --ros-args -p use_sim_time:=true' >/dev/null
  echo "LOOP_UP vp=$VP_IMAGE bridge=$BRIDGE_IMAGE"
}

run() {
  local tag=$1 logs
  logs=$(mkdir -p "${LOG_DIR:-./loop-logs}" && cd "${LOG_DIR:-./loop-logs}" && pwd)
  docker stop pov-bridge >/dev/null
  "$PY" -I - <<'EOF' 2>&1 | grep -v projection
import carla
c = carla.Client("localhost", 2000); c.set_timeout(30); w = c.get_world()
s = w.get_settings(); s.synchronous_mode = False; w.apply_settings(s); w.wait_for_tick(5)
left = list(w.get_actors().filter("vehicle.*")) + list(w.get_actors().filter("sensor.*"))
c.apply_batch_sync([carla.command.DestroyActor(a.id) for a in left], False)
w.wait_for_tick(5); w.wait_for_tick(5)
print("cleared", len(left))
EOF
  docker restart pov-vp >/dev/null
  (cd "$ROS2" && SCENARIO_FAST_START=1 exec "$PY" -I src/carla_bridge_bringup/scripts/config_carla.py \
    --map PoV_Highway --scenario highway_benchmark > "$logs/scenario-$tag.log" 2>&1) &
  # The scenario never exits by itself. Stop it when the run ends: the
  # observer ends with the hero, or the user presses Ctrl-C.
  trap "kill $! 2>/dev/null" EXIT   # $! expands now: the scenario PID
  # ponytail: both waits have no timeout; read the scenario log if one hangs.
  until grep -q -E "Running\.\.\.|Traceback" "$logs/scenario-$tag.log" 2>/dev/null; do sleep 1; done
  docker start pov-bridge >/dev/null
  # Not grep -q: under pipefail, an early grep exit makes docker die of SIGPIPE.
  until docker logs --since 1m pov-bridge 2>&1 | grep "hero_actor found" >/dev/null; do sleep 1; done
  "$PY" -I "$ROS2/loop/observer.py" > "$logs/observer-$tag.log" 2>&1
}

case ${1:-} in
  up) up ;;
  run) run "${2:?usage: loop_run.sh run <tag>}" ;;
  *) echo "usage: loop_run.sh up | run <tag>" >&2; exit 2 ;;
esac
