def visualize_all_waypoints(world, map, sampling_resolution):
    waypoints = map.generate_waypoints(distance=2.0)
    for waypoint in waypoints:
        world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,
                                color=carla.Color(r=255, g=0, b=0), life_time=120.0,
                                persistent_lines=True)
