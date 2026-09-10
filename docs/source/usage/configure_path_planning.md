# Path planning

IR-SIM includes grid- and sampling-based path planners for computing a collision-free path on an occupancy map. Planners are used **programmatically**: you build a map from the world, plan a path from the robot's state to its goal, and (optionally) let the robot follow it with a `dash` behavior or your own controller.

## Overview

Every planner follows the same four-step workflow:

1. **Build the map**: `env.get_map(resolution=...)` provides the environment map. A* and JPS use the map's existing grid resolution when a grid is present, rather than resampling it to this argument.
2. **Create the planner**: instantiate one planner from `irsim.lib.path_planners`.
3. **Plan**: `planner.planning(start, goal)` returns a NumPy array with shape `(2, N)` on success: row 0 is x and row 1 is y, in metres. Check the planner-specific failure result before drawing or following it.
4. **Draw / follow**: `env.draw_trajectory(trajectory)` overlays the path; feed it to a controller to follow it.

The start and goal come from the scene: `env.get_robot_state()` and `env.get_robot_info().goal`.

## Supported algorithms

| Planner | Import from `irsim.lib.path_planners` | Type | Use when |
| --- | --- | --- | --- |
| **A\*** | `AStarPlanner` | grid, 8-neighbour | you want a fast, optimal grid path |
| **JPS** | `JPSPlanner` | grid (optimised A\*) | reduce search work by jumping over intermediate grid cells |
| **RRT** | `RRT` | sampling | non-grid / Shapely obstacles; a feasible (not optimal) path |
| **RRT\*** | `RRTStar` | sampling | shorter, optimised paths via rewiring |
| **Informed RRT\*** | `InformedRRTStar` | sampling | faster convergence to the optimum after a first solution |
| **PRM** | `PRMPlanner` | sampling roadmap | connect sampled free-space configurations with a roadmap |

The planners query the environment map, which can combine grid occupancy and object geometry. Their collision approximations differ: A* and JPS use grid-cell geometry, PRM uses `robot_radius`, and the RRT family accepts a robot for footprint checks. A geometric path is not a guarantee of dynamically feasible or collision-free execution by a particular controller. Check footprint clearance and tracking behavior in the simulation.

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
if trajectory is not None and trajectory.shape[1] > 1:
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

:::{tab-item} Demonstration

```{image} https://raw.githubusercontent.com/IR-SIM/ir-sim-gifs/main/path_planning/path_planning.gif
:alt: A-star search expanding over an occupancy map before drawing its final path
:width: 500px
:align: center
```
:::

::::

Other planners use the same basic workflow, but differ in constructor options, failure results, and collision models. PRM rebuilds its samples and roadmap on each `planning()` call; this implementation does not automatically cache a roadmap across queries.

## Planner constructors

The first argument is always the map; sampling planners also take the robot (for collision radius) and tuning parameters.

```python
from irsim.lib.path_planners import (
    AStarPlanner,
    JPSPlanner,
    RRT,
    RRTStar,
    InformedRRTStar,
    PRMPlanner,
)

env_map = env.get_map(resolution=0.2)  # 0.1 is typical for the RRT family

AStarPlanner(env_map)
JPSPlanner(env_map)
RRT(env_map, robot=env.robot, expand_dis=1.5, max_iter=5000)
RRTStar(env_map, robot=env.robot, expand_dis=1.5, max_iter=5000)
InformedRRTStar(env_map, robot=env.robot, expand_dis=1.5, max_iter=5000)
PRMPlanner(env_map, robot_radius=env.robot.radius, n_sample=500, n_knn=10)
```

## Return values and path direction

Successful paths have shape `(2, N)` and are reconstructed from **goal to start**. Drawing is independent of this direction, but a waypoint-following controller usually needs start-to-goal order:

```python
if trajectory is not None and trajectory.shape[1] > 1:
    waypoints = trajectory[:, ::-1].T  # (N, 2), start → goal
```

| Planner | No path found |
| --- | --- |
| A* | A goal-only array with shape `(2, 1)` when the search is exhausted |
| JPS, RRT, RRT*, Informed RRT* | `None` |
| PRM | Empty NumPy array with shape `(2, 0)` |

The examples above use distinct start and goal positions, so a single-point result is not a usable connecting path. A* currently does not return an explicit failure flag: check that the path connects to the requested start grid cell before passing it to a controller. A single point can also represent a trivial start-equals-goal query.

For RRT-family planners, `len(planner.node_list)` reports tree size. Do not assume PRM exposes tree attributes. Sampling-based results depend on the random seed and search budget; optimality terminology describes algorithmic properties, not a promise that a finite run finds the continuous-space optimum.

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
