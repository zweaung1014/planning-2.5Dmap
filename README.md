# RRT* Path Planning on a 2.5D Map

A simulation of the RRT* (Rapidly-exploring Random Tree Star) algorithm running on a 2.5D map — a 2D grid where each cell stores an elevation (z) value.

## Setup

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Usage

```bash
python main.py
```

This runs the RRT* planner and opens a matplotlib window showing the exploration tree and final path.

## Configuration

Edit `config.py` to change:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MAP_SIZE_X` | 5.0 m | Map width |
| `MAP_SIZE_Y` | 5.0 m | Map height |
| `CELL_RESOLUTION` | 0.05 m | Grid cell size (5 cm) |
| `START` | (0, 0) | Robot start position |
| `GOAL` | (4.5, 4.5) | Goal position |
| `STEP_SIZE` | 0.2 m | RRT* expansion step |
| `SEARCH_RADIUS` | 0.5 m | Neighbor search radius for rewiring |
| `MAX_ITERATIONS` | 5000 | Max sampling iterations |
| `GOAL_TOLERANCE` | 0.1 m | Distance to goal considered "reached" |

## Adding Obstacles

In `main.py`, use the map's obstacle methods:

```python
env_map.set_obstacle_region(2.0, 2.0, 3.0, 3.0)  # rectangular region
env_map.set_obstacle(1.5, 3.0)                     # single cell
```

Obstacles are stored as cells with z-value of -1.

## Project Structure

```
├── config.py        # All configurable parameters
├── map2d5.py        # Map2D5 class (2.5D grid map)
├── rrt_star.py      # RRTStar class (algorithm implementation)
├── visualizer.py    # Visualizer class (matplotlib rendering)
├── main.py          # Entry point
└── requirements.txt # Dependencies (numpy, matplotlib)
```
