Path planning
=============

IR-SIM includes grid-based path planners for computing collision-free paths on occupancy maps. Planners are used programmatically; the simulation can then follow the path (e.g. with a dash behavior or custom control).

Supported algorithms
--------------------

- **A\*** — Classic grid A* with 8-neighbourhood.
- **JPS (Jump Point Search)** — Faster grid search with equivalent optimal paths.
- **RRT** — Rapidly-exploring Random Tree; no grid required, works with Shapely obstacles.
- **RRT\*** — Optimised RRT with rewiring for shorter paths.
- **Informed RRT\*** — RRT* with an informed search heuristic (ellipsoid) once an initial solution exists.

When a grid map is present (e.g. from ``obstacle_map`` in the world config), A* and JPS use grid occupancy for collision checks. RRT/RRT*/Informed RRT* can use the same grid or Shapely geometry.

Example usage
-------------

Scripts and YAML under **usage/20path_planning** demonstrate each planner:

- ``path_planning_astar.py`` — A* on a Perlin grid.
- ``path_planning_jps.py`` — Jump Point Search on the same setup.
- ``path_planning_rrt.py`` — RRT with optional grid + obstacles.
- ``path_planning_rrt_star.py`` — RRT*.
- ``path_planning_informed_rrt_star.py`` — Informed RRT*.

Use the shared ``path_planning.yaml`` (or equivalent) for world and robot; each script selects the planner and computes a path from the robot’s initial state to its goal. See the scripts and ``irsim.lib.path_planners`` for API details.
