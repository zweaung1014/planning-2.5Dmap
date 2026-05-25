"""Visualization of the 2.5D map, RRT* tree, and planned path."""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

from map2d5 import Map2D5
from rrt_star import Node


class Visualizer:
    """Renders the map, RRT* tree, and final path using matplotlib."""

    def __init__(self, map_env: Map2D5):
        self.map_env = map_env
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_xlim(0, map_env.size_x)
        self.ax.set_ylim(0, map_env.size_y)
        self.ax.set_aspect("equal")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_title("RRT* Path Planning on 2.5D Map")

    def draw_map(self):
        """Draw the map grid, showing obstacles in black."""
        obstacle_mask = self.map_env.grid == Map2D5.OBSTACLE
        self.ax.imshow(
            obstacle_mask,
            extent=[0, self.map_env.size_x, 0, self.map_env.size_y],
            origin="lower",
            cmap="Greys",
            alpha=0.3,
            vmin=0,
            vmax=1,
        )

    def draw_tree(self, nodes: list[Node]):
        """Draw the RRT* tree edges."""
        for node in nodes:
            if node.parent is not None:
                self.ax.plot(
                    [node.x, node.parent.x],
                    [node.y, node.parent.y],
                    color="lightskyblue",
                    linewidth=0.3,
                    alpha=0.5,
                )

    def draw_path(self, path: list[tuple[float, float]]):
        """Draw the final planned path."""
        if not path:
            return
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        self.ax.plot(xs, ys, color="red", linewidth=2.0, label="Path")

    def draw_start_goal(self, start: tuple[float, float], goal: tuple[float, float]):
        """Draw start and goal markers."""
        self.ax.plot(start[0], start[1], "go", markersize=10, label="Start")
        self.ax.plot(goal[0], goal[1], "r*", markersize=15, label="Goal")

    def show(self):
        """Display the plot."""
        self.ax.legend(loc="upper left")
        plt.tight_layout()
        plt.show()
