"""Configuration parameters for the 2.5D map RRT* simulation."""


# Map parameters
MAP_SIZE_X = 5.0  # meters
MAP_SIZE_Y = 5.0  # meters
CELL_RESOLUTION = 0.05  # meters per cell (5 cm)

# Robot start position (x, y) in meters
START = (0.0, 0.0)

# Goal position (x, y) in meters
GOAL = (4.5, 4.5)

# RRT* parameters
STEP_SIZE = 0.2  # meters
SEARCH_RADIUS = 0.5  # meters
MAX_ITERATIONS = 5000
GOAL_TOLERANCE = 0.1  # meters
