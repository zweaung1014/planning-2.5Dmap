# Height-Aware RRT* Planner

## Overview

This planner implements RRT* (Rapidly-exploring Random Tree Star) with elevation-aware cost computation, designed for a hopping robot navigating a 2.5D terrain map.

The key insight is that a hopping robot can traverse elevated terrain (unlike a wheeled robot), but doing so has an energy cost. The planner balances:
- **Distance cost**: longer paths are more expensive
- **Elevation cost**: jumping up/down incurs energy penalties
- **Hard limits**: some height differences are simply too large to jump

## Algorithm

### Standard RRT* Components

1. **Random sampling**: Points are uniformly sampled in the map bounds, with a 5% bias toward the goal to improve convergence.
2. **Nearest neighbor**: Finds the closest existing node (by Euclidean distance) to the sampled point.
3. **Steering**: Extends from the nearest node toward the sample by at most `STEP_SIZE` meters.
4. **Collision checking**: Verifies the edge is traversable (see below).
5. **Parent selection**: Chooses the lowest-cost parent from nearby nodes within `SEARCH_RADIUS`.
6. **Rewiring**: After adding a node, checks if nearby nodes would benefit from rerouting through the new node.

### Height-Aware Extensions

#### Edge Cost Function

The cost of traversing an edge between two nodes is:

$$\text{cost}(A \to B) = d_{xy}(A, B) + \sum_{i} w_i \cdot |\Delta z_i|$$

Where:
- $d_{xy}(A, B)$ is the Euclidean distance in the xy-plane
- $\Delta z_i = z_{i+1} - z_i$ is the height change between consecutive sample points along the edge
- $w_i = \alpha_{\text{uphill}}$ if $\Delta z_i > 0$ (jumping up)
- $w_i = \alpha_{\text{downhill}}$ if $\Delta z_i < 0$ (landing down)

The edge is sampled at half-cell resolution to capture intermediate terrain features.

#### Hard Jump Constraint

An edge is **impassable** (treated like an obstacle) if any consecutive sample pair along it has:

$$|\Delta z_i| > \text{MAX\_JUMP\_HEIGHT}$$

This models the physical limitation that the robot simply cannot make jumps above a certain height, regardless of the energy cost.

#### Collision Checking (Extended)

The `_collision_free()` method now checks two conditions along every edge:
1. No sampled point lies on an obstacle cell (z == -1)
2. No consecutive height difference exceeds `MAX_JUMP_HEIGHT`

If either condition fails, the edge is rejected.

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MAX_JUMP_HEIGHT` | 0.5 m | Maximum height difference the robot can traverse in a single hop. Edges exceeding this are impassable. |
| `ALPHA_UPHILL` | 1.0 | Cost multiplier for uphill jumps. Higher values make the planner prefer flat paths over elevated ones. |
| `ALPHA_DOWNHILL` | 0.5 | Cost multiplier for downhill jumps (landings). Lower than uphill because landing is generally easier than jumping up. |
| `STEP_SIZE` | 0.2 m | Maximum extension distance per RRT* iteration. |
| `SEARCH_RADIUS` | 0.5 m | Radius within which to search for better parents and rewiring candidates. |
| `MAX_ITERATIONS` | 5000 | Maximum number of sampling iterations before giving up. |
| `GOAL_TOLERANCE` | 0.1 m | Distance from goal at which the path is considered complete. |

## Tuning Guide

### Making the robot prefer flat paths (go around)
- Increase `ALPHA_UPHILL` and `ALPHA_DOWNHILL`
- The planner will accept longer 2D distances to avoid elevation changes

### Making the robot prefer short paths (hop over)
- Decrease `ALPHA_UPHILL` and `ALPHA_DOWNHILL` (toward 0)
- At 0, the planner ignores elevation entirely and finds the shortest 2D path

### Adjusting asymmetry
- `ALPHA_UPHILL > ALPHA_DOWNHILL`: jumping up costs more than landing down (default behavior — realistic for a hopping robot)
- `ALPHA_UPHILL == ALPHA_DOWNHILL`: symmetric cost for up and down

### Hard constraint
- Lower `MAX_JUMP_HEIGHT` to make more terrain impassable
- Set it very high (e.g., 100.0) to effectively disable the hard constraint and rely only on soft costs

## Architecture

```
config.py          ← All parameters in one place
map2d5.py          ← Map2D5: 2D grid with z-values, obstacle queries
rrt_star.py        ← RRTStar: sampling, steering, cost computation, rewiring
visualizer.py      ← Visualizer: matplotlib rendering
main.py            ← Entry point: build map → plan → visualize
```

### Key Classes

- **`Map2D5`**: Stores the terrain as a numpy array. Each cell holds an elevation (z in meters). Value -1 means obstacle. Provides `is_obstacle()`, `get_elevation()`, coordinate transforms.
- **`Node`**: Dataclass with `x`, `y`, `cost`, `parent`. Forms a tree structure.
- **`RRTStar`**: The planner. Owns the tree, implements the algorithm, computes height-aware costs.
- **`Visualizer`**: Draws grid, obstacles, tree edges, and final path.

## Future Extensions

- **Dynamic search radius**: Scale `SEARCH_RADIUS` with $\gamma \cdot (\log n / n)^{1/d}$ for asymptotic optimality guarantees
- **Anisotropic cost**: Factor in approach angle for the hopping robot
- **Kinodynamic constraints**: Limit consecutive jump heights based on momentum
- **Elevation-colored visualization**: Color-code the path by cumulative elevation cost
