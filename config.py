"""Configuration parameters for the 2.5D map A* simulation."""


# Map parameters
MAP_SIZE_X = 5.0  # meters
MAP_SIZE_Y = 5.0  # meters
CELL_RESOLUTION = 0.2  # meters per cell (10 cm)

# Robot start position (x, y) in meters
START = (1.8, 3.0)

# Goal position (x, y) in meters
GOAL = (3.2, 1.5)

# Height-aware planning parameters
MAX_JUMP_HEIGHT = 0.5  # meters; edges with dz > this are impassable
ALPHA_UPHILL = 1.0  # cost weight for uphill elevation changes
ALPHA_DOWNHILL = 0.5  # cost weight for downhill elevation changes (landing)
