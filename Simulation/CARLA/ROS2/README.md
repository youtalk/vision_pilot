# VisionPilot ⇄ CARLA (0.9.16 / 0.10) — ROS 2

In order to connect VisionPilot to CARLA for closed loop simulation and to not introduce carla_bridge dependency, two
nodes are required, one to publish vehicle speed and one to publish a command message so CARLA can drive the vehicle.

`carla_vehicle_speed_publisher` package publishes the vehicle speed to `/vehicle/speed` topic, and
`carla_control_publisher` publishes `ackermann_msgs/AckermannDriveStamped` to
`/carla/hero/ackermann_control_cmd`.

## CARLA 0.10

CARLA 0.10 publishes its sensors over ROS 2 natively, so the server must be built with `-DENABLE_ROS2=ON` and started
with `--ros2`. `--rmw=cyclonedds --ros-domain-id=1` is what the bench uses, so the X5H board shares one domain, and it
is verified to publish every sensor topic. Read "Bench host firewall" below before you conclude that it does not. The rig is described by `config/carla10.json`, the 0.10 sibling of `config/carla916.json`:

- `map` selects the town. 0.10 ships the `*_Opt` variants, so it is `Town04_Opt` rather than `Town04`.
- `attributes.ros2_ackermann_control` must be `True`. Without it the server binds the `CarlaEgoVehicleControl`
  subscriber instead of the Ackermann one and silently drops every `ackermann_control_cmd` message — the ego simply
  never moves, with no error on either side.
- `spawn_index` is `5`. Town04_Opt spawn indices changed meaning in 0.10: the old default sits inside static geometry,
  and several low-curvature candidates are open paved aprons with no lane markings on either side. Indices 0-7 are
  middle lanes of the multi-lane highway.

### Steering sign

VisionPilot reports `+cte` as "ego right of path" and emits its tyre angle in the same sense, so a positive angle
steers right. CARLA's native ROS 2 Ackermann path uses that convention too rather than REP-103: the incoming
`steering_angle` is assigned straight to `AckermannControl.steer`, the Ackermann controller only rescales it by the
vehicle's maximum steering angle, and CARLA documents the field as "Desired steer (rad). Positive value is to the
right." The two conventions already agree, so `carla_control_publisher` passes the value through unchanged. Its
`steering_sign` parameter exists only for a differently-signed source; setting it to `-1.0` turns lane keeping into
positive feedback.

### VisionPilot configuration

The `config/visionpilot*.carla.conf` files are bind-mounted read-only over VisionPilot's own configs at run time so the
in-tree templates stay pristine. `config/H_carla.yaml` must be mounted over `VisionPilot/config/H.yaml`: VisionPilot
loads that filename at runtime, and mounting only `homography_C_matrix.yaml` leaves it projecting AutoSteer waypoints
with the default OpenLane homography. `gen_carla_C_matrix.py` regenerates `config/homography_C_matrix.yaml` from
`config/H_carla.yaml` whenever the camera rig changes.

### Bench host firewall

The bench host `rog-amd` runs an nftables ruleset (`table inet x5h`) whose input chain has `policy drop`. That chain
accepts loopback and a short list of ports on the bench NIC. It does not accept DDS.

Discovery packets are fresh multicast, so the `ct state established,related` rule never matches them. The failure is
silent. CARLA creates every publisher and binds the domain's ports, and no consumer ever sees a topic. Fast-DDS gives
the same result, because a firewall does not depend on the middleware.

Add this rule to the input chain of `/etc/nftables.conf` on the bench host:

```
  # Native ROS 2 / DDS on the bench LAN. CycloneDDS domain 1 uses UDP 7650-7651
  # for multicast discovery and 7660-7679 for the per-participant unicast ports
  # (MaxAutoParticipantIndex 9). SPDP arrives as fresh multicast, not as
  # conntrack-established, so the `ct state` rule above never matches it.
  iifname $BIF udp dport 7650-7679 accept
```

Then run `sudo nft -f /etc/nftables.conf`.

A host that sends multicast to its own NIC receives it back through the input hook, with that NIC as the input
interface rather than `lo`. So this rule is needed even when the server and the consumer both run on `rog-amd`.

To make sure that the path works, run `ros2 topic list` from a `--net=host` container on domain 1. Use a small `rclpy`
script instead if the CLI hangs, which it does on this host.
