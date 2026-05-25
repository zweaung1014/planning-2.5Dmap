"""2.5D Map representation: a 2D grid with elevation (z) values per cell."""

import numpy as np


class Map2D5:
    """A 2.5D map stored as a 2D grid of elevation values.

    Each cell holds a z-value (elevation in meters). A value of -1 indicates
    an obstacle. The map origin (0, 0) is at the bottom-left corner.
    """

    OBSTACLE = -1.0

    def __init__(self, size_x: float, size_y: float, resolution: float, default_z: float = 0.0):
        self.size_x = size_x
        self.size_y = size_y
        self.resolution = resolution
        self.cols = int(round(size_x / resolution))
        self.rows = int(round(size_y / resolution))
        self.grid = np.full((self.rows, self.cols), default_z, dtype=np.float64)

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        """Convert world coordinates (meters) to grid indices (row, col)."""
        col = int(x / self.resolution)
        row = int(y / self.resolution)
        col = max(0, min(col, self.cols - 1))
        row = max(0, min(row, self.rows - 1))
        return row, col

    def grid_to_world(self, row: int, col: int) -> tuple[float, float]:
        """Convert grid indices to world coordinates (center of cell)."""
        x = (col + 0.5) * self.resolution
        y = (row + 0.5) * self.resolution
        return x, y

    def is_obstacle(self, x: float, y: float) -> bool:
        """Check if the world position (x, y) is an obstacle."""
        if x < 0 or x >= self.size_x or y < 0 or y >= self.size_y:
            return True
        row, col = self.world_to_grid(x, y)
        return self.grid[row, col] == self.OBSTACLE

    def get_elevation(self, x: float, y: float) -> float:
        """Get the elevation (z value) at world position (x, y)."""
        row, col = self.world_to_grid(x, y)
        return self.grid[row, col]

    def set_obstacle(self, x: float, y: float):
        """Mark the cell at world position (x, y) as an obstacle."""
        row, col = self.world_to_grid(x, y)
        self.grid[row, col] = self.OBSTACLE

    def set_obstacle_region(self, x_min: float, y_min: float, x_max: float, y_max: float):
        """Mark a rectangular region as obstacles."""
        r_min, c_min = self.world_to_grid(x_min, y_min)
        r_max, c_max = self.world_to_grid(x_max, y_max)
        self.grid[r_min:r_max + 1, c_min:c_max + 1] = self.OBSTACLE

    def is_within_bounds(self, x: float, y: float) -> bool:
        """Check if (x, y) is within map boundaries."""
        return 0 <= x < self.size_x and 0 <= y < self.size_y
