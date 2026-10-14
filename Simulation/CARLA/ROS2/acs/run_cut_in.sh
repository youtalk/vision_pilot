#!/usr/bin/env bash
#
# VisionPilot against a cut-in on Town04_Opt's highway, run by
# autoware_carla_scenario with the ego on ROS 2 (README.md).
#
# Usage:
#   ./run_cut_in.sh [scenario override ...]
#
# Needs a CARLA 0.10 server on Town04_Opt (CARLA_HOST:CARLA_PORT), the
# visionpilot:gpu-ros2 image (VisionPilot/docker/build.sh --gpu --ros2) and an
# autoware_carla_scenario checkout with the ros2 extra synced (ACS_DIR). Extra
# arguments are Hydra overrides for the scenario, e.g. scenario.npc_gap_m=15.
#
# Environment:
#   ACS_DIR         autoware_carla_scenario checkout (default: ../../../../../autoware_carla_scenario)
#   CARLA_HOST      default 127.0.0.1
#   CARLA_PORT      default 2000
#   ROS_DOMAIN_ID   default 7, so that nothing else on this host joins in
#   VP_IMAGE        default visionpilot:gpu-ros2
#   VP_KEEP=1       leave the VisionPilot container running afterwards

set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROS2_DIR="$(cd "$HERE/.." && pwd)"
ACS_DIR="${ACS_DIR:-$(cd "$HERE/../../../../.." && pwd)/autoware_carla_scenario}"
CARLA_HOST="${CARLA_HOST:-127.0.0.1}"
CARLA_PORT="${CARLA_PORT:-2000}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
VP_IMAGE="${VP_IMAGE:-visionpilot:gpu-ros2}"
VP_NAME="visionpilot-acs-${ROS_DOMAIN_ID}"
export CYCLONEDDS_URI="file://$HERE/cyclonedds.xml"

if [ ! -f "$ACS_DIR/pyproject.toml" ]; then
    echo "Error: no autoware_carla_scenario checkout at ACS_DIR=$ACS_DIR" >&2
    exit 1
fi

# VisionPilot first: loading the models takes a while, and the ego is held on
# its brakes until the first command arrives.
docker rm -f "$VP_NAME" >/dev/null 2>&1 || true
docker run -d --name "$VP_NAME" --gpus all --network host --ipc host \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e ROS_DOMAIN_ID \
    -e CYCLONEDDS_URI=file:///acs/cyclonedds.xml \
    -v "$HERE/cyclonedds.xml:/acs/cyclonedds.xml:ro" \
    -v "$HERE/config/visionpilot.acs.conf:/usr/share/visionpilot/config/vision_pilot.conf:ro" \
    -v "$ROS2_DIR/config/visionpilot_ros2.carla.conf:/usr/share/visionpilot/config/vision_pilot_ros2.conf:ro" \
    -v "$ROS2_DIR/config/H_carla.yaml:/usr/share/visionpilot/config/H.yaml:ro" \
    -v "$ROS2_DIR/config/homography_C_matrix.yaml:/usr/share/visionpilot/config/homography_C_matrix.yaml:ro" \
    -v visionpilot-engine-cache:/cache \
    "$VP_IMAGE" --no-window >/dev/null
if [ -z "${VP_KEEP:-}" ]; then
    trap 'docker logs "$VP_NAME" > "${VP_LOG:-/tmp/$VP_NAME.log}" 2>&1 || true; docker rm -f "$VP_NAME" >/dev/null 2>&1 || true' EXIT
fi

echo "Waiting for VisionPilot to load its models ..."
for _ in $(seq 300); do
    if docker logs "$VP_NAME" 2>&1 | grep -q "\[AutoSpeed\] Ready"; then
        break
    fi
    if ! docker ps -q --filter "name=^${VP_NAME}$" | grep -q .; then
        docker logs "$VP_NAME" >&2 || true
        echo "Error: VisionPilot exited" >&2
        exit 1
    fi
    sleep 1
done

# The cut-in from the left on lanelet 79016, where Town04_Opt's spawn point 5
# stands, inside OpenDRIVE road 41 (README.md, "The Town04 map"). The NPC's
# rear bumper starts 8 m ahead of the ego's front one, along the ego's lane.
cd "$ACS_DIR"
uv run --no-sync scenario \
    "hydra.searchpath=[file://$HERE/conf]" \
    ros2=visionpilot \
    map=town04_opt \
    scenario=cut_in/left \
    ego.spawn_lanelet_id=79016 \
    ego.spawn_s=640.0 \
    scenario.npc_lanelet_id=77298 \
    scenario.npc_gap_m=8 \
    server.host="$CARLA_HOST" \
    server.port="$CARLA_PORT" \
    "$@"
