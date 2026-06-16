# Path planning

IR-SIM includes grid- and sampling-based path planners for computing a collision-free path on an occupancy map. Planners are used **programmatically**: you build a map from the world, plan a path from the robot's state to its goal, and (optionally) let the robot follow it with a `dash` behavior or your own controller.

## Overview

Every planner follows the same four-step workflow:

1. **Build the map** — `env.get_map(resolution=...)` returns the occupancy grid the planner searches (`resolution` is the planning cell size in meters).
2. **Create the planner** — instantiate one planner from `irsim.lib.path_planners`.
3. **Plan** — `planner.planning(start, goal)` returns a trajectory (list of `[x, y]` points), or `None` if no path is found.
4. **Draw / follow** — `env.draw_trajectory(trajectory)` overlays the path; feed it to a controller to follow it.

The start and goal come from the scene: `env.get_robot_state()` and `env.get_robot_info().goal`.

## Supported algorithms

| Planner | Import from `irsim.lib.path_planners` | Type | Use when |
| --- | --- | --- | --- |
| **A\*** | `AStarPlanner` | grid, 8-neighbour | you want a fast, optimal grid path |
| **JPS** | `JPSPlanner` | grid (optimised A\*) | same paths as A\*, faster on open grids |
| **RRT** | `RRT` | sampling | non-grid / Shapely obstacles; a feasible (not optimal) path |
| **RRT\*** | `RRTStar` | sampling | shorter, optimised paths via rewiring |
| **Informed RRT\*** | `InformedRRTStar` | sampling | faster convergence to the optimum after a first solution |
| **PRM** | `PRMPlanner` | sampling roadmap | many queries against the same static map |

Grid planners (A\*, JPS) and PRM search the grid occupancy from `obstacle_map`; RRT / RRT\* / Informed RRT\* can use the same grid or Shapely obstacle geometry.

## Quick example

Plan an A\* path on a Perlin-noise grid and draw it. Pass `show_animation=True` to watch the search expand live.

::::{tab-set}

:::{tab-item} Python
```python
import irsim
from irsim.lib.path_planners import AStarPlanner

env = irsim.make("path_planning.yaml", save_ani=False, full=False)

# 1. occupancy grid from the world's obstacle_map
env_map = env.get_map(resolution=0.2)

# 2. planner
planner = AStarPlanner(env_map)

# 3. plan from the robot's state to its goal
robot_state = env.get_robot_state()
goal_xy = env.get_robot_info().goal[:2, 0].tolist()
trajectory = planner.planning(robot_state, goal_xy, show_animation=True)

# 4. draw the path as a red line
if trajectory is not None:
    env.draw_trajectory(trajectory, traj_type="r-")

env.end(5)
```
:::

:::{tab-item} YAML (path_planning.yaml)
```yaml
world:
  height: 20
  width: 30
  step_time: 0.1
  obstacle_map:           # procedural occupancy grid, built at load time
    name: perlin
    resolution: 0.1
    complexity: 0.08
    fill: 0.15
    fractal: 1
    attenuation: 0.5
    seed: 56

robot:
  - kinematics: {name: 'diff'}
    shape: {name: 'circle', radius: 0.2}
    state: [2, 2, 0]
    goal: [24, 16, 0]
    behavior: {name: 'dash'}
```
:::

::::

Swap `AStarPlanner` for any other planner — only the constructor changes; `planning()` and `draw_trajectory()` stay the same.

## Planner constructors

The first argument is always the map; sampling planners also take the robot (for collision radius) and tuning parameters.

```python
from irsim.lib.path_planners import (
    AStarPlanner, JPSPlanner, RRT, RRTStar, InformedRRTStar, PRMPlanner,
)

env_map = env.get_map(resolution=0.2)   # 0.1 is typical for the RRT family

AStarPlanner(env_map)
JPSPlanner(env_map)
RRT(env_map, robot=env.robot, expand_dis=1.5, max_iter=5000)
RRTStar(env_map, robot=env.robot, expand_dis=1.5, max_iter=5000)
InformedRRTStar(env_map, robot=env.robot, expand_dis=1.5, max_iter=5000)
PRMPlanner(env_map, robot_radius=env.robot.radius, n_sample=500, n_knn=10)
```

`planning(start, goal, show_animation=False)` returns a list of `[x, y]` points or `None`. For the sampling planners, `planner.end.cost` and `len(planner.node_list)` report the path cost and tree size.

## Runnable examples

Scripts under **`usage/20path_planning`** run every planner on the shared `path_planning.yaml`:

| Script | Planner |
| --- | --- |
| `path_planning_astar.py` | A\* |
| `path_planning_jps.py` | Jump Point Search |
| `path_planning_rrt.py` | RRT |
| `path_planning_rrt_star.py` | RRT\* |
| `path_planning_informed_rrt_star.py` | Informed RRT\* |
| `path_planning_prm.py` | PRM |

See the `irsim.lib.path_planners` section of the {doc}`API Reference <../api/index>` for the full planner API.
