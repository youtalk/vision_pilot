#!/usr/bin/env python3
"""Walk the ego along the lane centre so lane detection is measured while moving.

Teleports the ego from waypoint to waypoint with physics off, so the viewpoint is
in-lane and moving without a controller, a traffic manager or the Ackermann path
taking part. That isolates perception: whatever the lane fit rate turns out to be,
it cannot be blamed on a planner or on the ego sitting still at its spawn pose.

  lane_walk.py [seconds] [speed_mps]
"""
import sys
import time
import carla

SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 140.0
SPEED = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
DT = 0.1

client = carla.Client("127.0.0.1", 2000); client.set_timeout(30.0)
world = client.get_world()
bl = world.get_blueprint_library()

bp = bl.filter("vehicle.lincoln.mkz")[0]
bp.set_attribute("role_name", "hero"); bp.set_attribute("ros_name", "hero")
veh = world.spawn_actor(bp, world.get_map().get_spawn_points()[5])
veh.set_simulate_physics(False)

cbp = bl.filter("sensor.camera.rgb")[0]
cbp.set_attribute("ros_name", "main_cam"); cbp.set_attribute("role_name", "main_cam")
for key, val in (("image_size_x", "1280"), ("image_size_y", "720"), ("fov", "60.0"), ("sensor_tick", "0.1")):
    cbp.set_attribute(key, val)
cam = world.spawn_actor(cbp, carla.Transform(carla.Location(x=1.25, z=1.58)), attach_to=veh)
cam.enable_for_ros()
print("WALK ego=%d cam=%d" % (veh.id, cam.id), flush=True)

wp = world.get_map().get_waypoint(veh.get_location(), project_to_road=True,
                                  lane_type=carla.LaneType.Driving)
t0 = time.time()
steps = 0
while time.time() - t0 < SECS:
    nxt = wp.next(max(0.05, SPEED * DT))
    if not nxt:
        print("WALK lane ended after %d steps" % steps, flush=True)
        break
    wp = nxt[0]
    tf = carla.Transform(carla.Location(wp.transform.location.x, wp.transform.location.y,
                                        wp.transform.location.z + 0.1), wp.transform.rotation)
    veh.set_transform(tf)
    steps += 1
    time.sleep(DT)

print("WALK steps=%d distance_m=%.1f" % (steps, steps * SPEED * DT), flush=True)
cam.stop(); cam.destroy(); veh.destroy()
print("WALK done", flush=True)
