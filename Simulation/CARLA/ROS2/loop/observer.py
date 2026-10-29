"""Log ego speed, scenario gaps and ego collisions once per simulated second."""
import math, sys, time, carla
c = carla.Client("localhost", 2000); c.set_timeout(30)
w = c.get_world(); m = w.get_map()
hero = None
while hero is None:
    hero = next((a for a in w.get_actors().filter("vehicle.*") if a.attributes.get("role_name") == "hero"), None)
    time.sleep(0.5)
col = w.spawn_actor(w.get_blueprint_library().find("sensor.other.collision"), carla.Transform(), attach_to=hero)
col.listen(lambda e: print(f"EGO_COLLISION with {e.other_actor.type_id} impulse={e.normal_impulse.length():.0f}", flush=True))
last = -1
while hero.is_alive:
    snap = w.wait_for_tick(30)
    t = snap.elapsed_seconds
    if int(t) == last: continue
    last = int(t)
    v = hero.get_velocity(); spd = math.sqrt(v.x**2 + v.y**2 + v.z**2)
    wp = m.get_waypoint(hero.get_location())
    lane_off = hero.get_location().distance(wp.transform.location)
    near = []
    for a in w.get_actors().filter("vehicle.*"):
        if a.id == hero.id: continue
        d = a.get_location().distance(hero.get_location())
        if d < 120:
            awp = m.get_waypoint(a.get_location())
            if awp.road_id == wp.road_id and awp.lane_id == wp.lane_id and awp.s > wp.s:
                av = a.get_velocity(); near.append((round(awp.s - wp.s, 1), round(math.sqrt(av.x**2 + av.y**2), 1), a.type_id.split(".")[-1]))
    near.sort()
    print(f"t={t:7.1f} s={wp.s:7.0f} lane={wp.lane_id} off={lane_off:4.2f} v={spd:5.2f} ahead_same_lane={near[:2]}", flush=True)
