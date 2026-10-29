# Closed loop on the CARLA 0.10 package

These two files run VisionPilot in a closed loop against the PoV_Highway scenarios on a CARLA 0.10 (`ue58-dev`) Shipping package. A closed loop means that VisionPilot drives the hero car in CARLA. The editor server does not work for this, because it publishes only `/clock`. Use a package.

- `loop_run.sh` creates the two containers and does one run.
- `observer.py` logs the hero speed, the gap to the cars ahead and each hero collision, once per simulated second.

## Before the first run

1. Build the GPU image: `VisionPilot/docker/build.sh --gpu --ros2`. It makes `visionpilot:gpu-ros2`.
2. Make a Python 3.12 virtual environment and install the `carla-0.10.0-cp312` wheel from the package. Set `PY` to its `python`.
3. Unzip the same wheel to a directory and set `CARLA_WHEEL_DIR` to it. The bridge container loads `carla` from there.
4. Start the CARLA server from the package: `Linux/CarlaUnreal.sh --ros2 --rmw=cyclonedds --ros-domain-id=42`. The containers use ROS domain 42.
5. Run `bash loop_run.sh up`. It adds CycloneDDS to the VisionPilot image and creates the containers `pov-vp` and `pov-bridge`. If `install/` is missing, it first builds the ROS 2 packages in this directory. It prints `LOOP_UP`.

VisionPilot shows its HUD in a window. If the window does not open, run `xhost +local:` on the desktop.

## One run

```bash
bash loop_run.sh run cut35
```

The logs go to `LOG_DIR` (default `./loop-logs`): `scenario-<tag>.log` and `observer-<tag>.log`. Other variables, for example `CUTOUT_TRIGGER_DIST` and `SCENARIO_FAST_START_M`, go to the scenario.

## Run order

The script starts the scenario first, then the bridge, then the observer. The bridge starts only after the scenario prints `Running...`. The speed publisher in the bridge looks for the hero only once, at its start. If the bridge starts first, it keeps a hero that the scenario then destroys. VisionPilot then gets no speed and accelerates without limit.

The script also destroys all vehicles and sensors before each run. A crashed run can leave parked Traffic Manager cars on the road, and they stop the lead car.
