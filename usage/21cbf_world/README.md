# CBF World Example

This folder contains standard control barrier function (CBF) controller examples。

## What This Example Does

This folder has two variants: a **distance-based CBF** and a **collision-cone CBF (C3BF)**. The controller first builds a nominal goal-seeking command `u_nom`, then solves a QP that stays close to `u_nom` while satisfying safety and input limits:

```text
minimize    ||u - u_nom||^2
subject to  CBF safety constraints
            input bounds
```

### CBF (`cbf_qp.py` / `cbf_world.py`)


Safety uses a **circular distance barrier** for each obstacle (combined radii plus margin):

```text
h = ||p - p_obs||^2 - d_safe^2
h_dot + alpha * h >= 0
```

Supported robots:

- `omni`: control `u = [vx, vy]`
- `diff`: control `u = [v, omega]` (lookahead mapping for the barrier, same idea as in `cbf_qp.py`)

`cbf_world.py` reads `robot.kinematics` from the YAML and passes it into `CBFQPController`.

### C3BF (`c3bf_qp.py` / `c3bf_world.py`)

This follows the collision-cone idea from
[*A Collision Cone Approach for Control Barrier Functions*](https://arxiv.org/abs/2403.07043).
Compared with the distance CBF above, it also encodes whether **relative velocity** points into the obstacle’s **collision cone**, not only whether centers are far enough apart.

For one obstacle, with

```text
p_rel = p_obs - p_robot
v_rel = v_robot - v_obs
r     = r_robot + r_obs + margin
```

this example uses the barrier

```text
h_cc = -p_rel^T v_rel + sqrt(||p_rel||^2 - r^2) ||v_rel||
```

- `h_cc > 0`: relative velocity is outside the cone (safer heading in that sense)
- `h_cc < 0`: motion is toward a collision direction in the cone sense

Because `h_cc` is nonlinear in the control, `c3bf_qp.py` **linearizes** it around the current command and imposes affine constraints in the QP. If that is unreliable (e.g. already inside the cone or too close), it **falls back** to the same distance CBF as `cbf_qp.py` for that obstacle.

IR-SIM uses **velocity** inputs, not the paper’s acceleration-controlled unicycle:

- `omni`: `v_robot = [vx, vy]`
- `diff`: lookahead point; planar velocity is `J [v, omega]`

`c3bf_world.py` reads `robot.kinematics` from the YAML and passes it into `C3BFQPController`.

## Requirements

This example uses `cvxpy` to solve the QP.

Install it in your IR-SIM Python environment if needed:

```bash
python -m pip install cvxpy
```

## How To Run

Run the default scene:

```bash
python usage/21cbf_world/cbf_world.py
```

Run the collision-cone version:

```bash
python usage/21cbf_world/c3bf_world.py
```

If you use a conda environment, activate it first or call that Python directly.

## Current Obstacle Support

The current CBF is designed for circular obstacles because it uses:

- obstacle position
- obstacle radius
- obstacle velocity

So this example works best with `circle` obstacles.

For `rectangle` or `polygon` obstacles, the current controller is not geometrically exact. A future extension would use a closest-point or convex-distance CBF.

## Notes

- If the QP solver cannot find a valid solution, the controller currently returns zero control input.
- If the robot appears stuck, that may mean either:
  - the QP became infeasible, or
  - zero control is still the safest feasible action.
- `c3bf_qp.py` is usually more meaningful in scenes with dynamic circular obstacles, because it explicitly uses obstacle velocity and collision-cone geometry.
