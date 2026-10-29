"""Base scenario definition for PoV_Highway."""
import carla
from scenarios.scenario_base import CarlaScenario


class PoVHighwayBase(CarlaScenario):
    """Base configuration for PoV_Highway (Autoware Simulation WG 15km Highway)."""
    map_name = "PoV_Highway"
    scenario_name = "base"
    description = "PoV_Highway baseline scenario"
    hero_type = "vehicle.lincoln.mkz"
    hero_id = "hero"
    hero_spawn_index = 0
    # (road, lane, s): the right lane of road 2 at its start. The ue58-dev
    # import of this map has no spawn points, so the ego spawns on the lane.
    hero_spawn_waypoint = (2, -2, 10.0)
    weather = carla.WeatherParameters.ClearNoon

