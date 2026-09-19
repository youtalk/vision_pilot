#!/usr/bin/env python3
"""Chase camera on the hero, stream 1 of the demo recording.

  demo_cam.py <out-dir> [--seconds 120] [--host 127.0.0.1] [--port 2000]
              [--hero-timeout 120]

A raw CARLA client like survey_ring.py and lap_gate.py: it runs in the host
carla venv, not in visionpilot:si, and it owns nothing but its own camera.
run-d6.sh owns the run, and the server, the world settings, the weather and the
hero all belong to config_carla.py, so this attaches a sensor to the hero that
is already there and listens. It never spawns a vehicle and never calls
set_weather.

It also never calls world.tick(). config_carla.py holds the world in
synchronous mode and drives the clock itself (config_carla.py:237, and see
lap_gate.find_hero), so a second ticker would advance the simulation twice per
frame and the whole run would drive at double speed.

One JPEG per frame plus an index.csv of arrival times, which is all the
composer reads to place this pane in time.

Prints DEMO_CAM ready once the camera is attached, then
DEMO_CAM n=<frames> dir=<dir>, or DEMO_CAM_FAIL reason=<slug>.
"""
import argparse
import os
import signal
import sys
import time

import carla
from demo_streams import open_index, write_row
from lap_gate import find_hero

# The chase view, in the hero's own frame: behind it, above it, tilted down.
# These three numbers are the whole framing of the pane and are meant to be
# tuned at the bench without reading the rest of this file. 6 m back and 2.8 m
# up with 15 degrees of down-pitch puts the car in the lower third and the road
# it is braking on above it.
CHASE_BACK_M = 6.0
CHASE_UP_M = 2.8
CHASE_PITCH_DEG = -15.0
CHASE_FOV_DEG = 90.0

WIDTH = 1280
HEIGHT = 720
# 20 Hz is what sensor_tick asks for. The server's fixed_delta_seconds is the
# real ceiling (config_carla.py runs 10 Hz), so the delivered rate can be half
# of this; index.csv carries the times that actually happened, so the composer
# never has to trust the nominal rate.
FPS = 20.0


def _on_sigterm(signum, frame):
    """record-demo.sh stops the recorders with SIGTERM.

    Without this the process dies in place, the camera is never detached, and
    the leaked sensor stays attached to the hero for the rest of the CARLA
    server's life. lum_ab.sh has already been bitten by a leaked actor from
    exactly this hole (lane_walk.py:35).
    """
    raise SystemExit(0)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("out")
    p.add_argument("--seconds", type=float, default=120.0)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=2000)
    p.add_argument("--hero-timeout", type=float, default=120.0)
    a = p.parse_args()
    # A non-positive --seconds records an empty directory and still exits 0.
    # That reads as a broken camera to whoever composes the reel, hours later
    # and without the bench in front of them, so refuse it while it is cheap.
    if a.seconds <= 0:
        print("DEMO_CAM_FAIL reason=bad_seconds", flush=True)
        return 1
    try:
        os.makedirs(a.out, exist_ok=True)
    except OSError:
        print("DEMO_CAM_FAIL reason=bad_out", flush=True)
        return 1

    client = carla.Client(a.host, a.port)
    client.set_timeout(30.0)
    try:
        world = client.get_world()
    except RuntimeError:
        print("DEMO_CAM_FAIL reason=no_server", flush=True)
        return 1

    # find_hero() blocks on a world snapshot, so this loop needs no sleep of its
    # own; it also raises rather than returning when no tick arrives at all.
    hero = None
    deadline = time.time() + a.hero_timeout
    try:
        while hero is None and time.time() < deadline:
            hero = find_hero(world)
    except RuntimeError:
        hero = None
    if hero is None:
        print("DEMO_CAM_FAIL reason=no_hero", flush=True)
        return 1

    cbp = world.get_blueprint_library().filter("sensor.camera.rgb")[0]
    for key, val in (("image_size_x", str(WIDTH)), ("image_size_y", str(HEIGHT)),
                     ("fov", str(CHASE_FOV_DEG)), ("sensor_tick", f"{1.0 / FPS:.4f}")):
        cbp.set_attribute(key, val)
    # Deliberately no ros_name and no enable_for_ros(): this camera must not
    # become a DDS topic. A second 1280x720 bgra8 publisher is 3.69 MB per
    # sample, which is the exact load jpeg_bridge.py exists to keep off the
    # bench LAN, and the chase view is for the video only.
    tf = carla.Transform(carla.Location(x=-CHASE_BACK_M, z=CHASE_UP_M),
                         carla.Rotation(pitch=CHASE_PITCH_DEG))
    try:
        cam = world.spawn_actor(cbp, tf, attach_to=hero)
    except RuntimeError:
        print("DEMO_CAM_FAIL reason=no_camera", flush=True)
        return 1

    index = open_index(os.path.join(a.out, "index.csv"))
    n = 0

    def on_image(img):
        # bench_time is the arrival time on this host, not img.timestamp: that
        # one is CARLA's simulation clock, which shares no origin with the
        # board's clock or with the fault instant.
        # CARLA delivers on one sensor thread, so n needs no lock.
        nonlocal n
        now = time.time()
        name = f"{n:06d}.jpg"
        # save_to_disk picks its codec from the extension, so .jpg is what makes
        # this a JPEG rather than the PNG the CARLA examples write.
        img.save_to_disk(os.path.join(a.out, name))
        write_row(index, name, now)
        n += 1

    cam.listen(on_image)
    signal.signal(signal.SIGTERM, _on_sigterm)
    print("DEMO_CAM ready", flush=True)
    try:
        end = time.time() + a.seconds
        while time.time() < end:
            # The camera delivers on its own thread; this one only has to keep
            # the process alive.
            time.sleep(0.2)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        # Never the hero: another process owns it. A failed cleanup must not
        # mask the count below, but it must not be silent either.
        for label, fn in (("cam.stop", cam.stop), ("cam.destroy", cam.destroy)):
            try:
                fn()
            except RuntimeError as exc:
                print(f"DEMO_CAM cleanup_failed {label}: {exc}", flush=True)
        index.close()
    print(f"DEMO_CAM n={n} dir={a.out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
