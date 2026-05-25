"""Entry point for the RRT* 2.5D map planning simulation."""

import config
from map2d5 import Map2D5
from rrt_star import RRTStar
from visualizer import Visualizer


def main():
    # Build map
    env_map = Map2D5(
        size_x=config.MAP_SIZE_X,
        size_y=config.MAP_SIZE_Y,
        resolution=config.CELL_RESOLUTION,
    )

    # (Optional) Add obstacles here for testing, e.g.:
    # env_map.set_obstacle_region(2.0, 2.0, 3.0, 3.0)

    # Plan path
    planner = RRTStar(
        map_env=env_map,
        start=config.START,
        goal=config.GOAL,
        step_size=config.STEP_SIZE,
        search_radius=config.SEARCH_RADIUS,
        max_iterations=config.MAX_ITERATIONS,
        goal_tolerance=config.GOAL_TOLERANCE,
    )
    path = planner.plan()

    if path is None:
        print("No path found.")
    else:
        print(f"Path found with {len(path)} waypoints.")

    # Visualize
    vis = Visualizer(env_map)
    vis.draw_map()
    vis.draw_tree(planner.nodes)
    if path:
        vis.draw_path(path)
    vis.draw_start_goal(config.START, config.GOAL)
    vis.show()


main()
