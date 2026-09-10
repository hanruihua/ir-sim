# From commands to motion

Change a velocity command and see how it becomes a trajectory. This lab connects the state and action conventions in [Concepts](../concepts.md) to the differential-drive update used by IR-SIM.

## Explore the model

The default scene starts at `(0, 0)`. Launch Python and click Run; the default duration is Continuous, so you can keep adjusting linear speed and yaw rate until you pause or reach the environment's completion or safety limit. To reproduce the numerical checkpoint below, select **4 s** before running and keep the command constant. Edit the initial heading and integration step in the YAML, then apply it. Scrub time to inspect previously computed states. The axes use metres and the same scale in both directions.

```{raw} html
<div class="kinematics-lab" data-kinematics-lab></div>
```

This widget runs the actual IR-SIM package in Pyodide, using the same component as the [Online Playground](../playground/index.md). The image comes directly from IR-SIM's Matplotlib renderer, including its robot, goal, and trajectory styles. The default scene is noise-free. Python handles simulation and rendering; JavaScript displays captured frames. The equations and runnable Python example below remain available if the runtime cannot load.

### Three experiments

1. Select slider control and set the yaw rate to `0`. The trajectory is straight and agrees with the analytic solution, apart from floating-point rounding.
2. Set the linear speed to `0`. Position stays fixed while the heading changes: a differential-drive robot can turn in place.
3. Select Free motion and apply its YAML, then compare `step_time: 0.2` with `step_time: 0.05`. Run each for **four simulated seconds** and compare the final positions with the analytic solution below, not after the same number of steps.

## State, frames, and units

| Quantity | Meaning | Unit |
| --- | --- | --- |
| `x`, `y` | Position in the world frame | m |
| `theta` | Heading from world +x toward +y; positive counterclockwise | rad |
| `v` | Forward speed in the robot's frame | m/s |
| `omega` | Yaw rate; positive counterclockwise | rad/s |
| `dt` | Simulated duration of one integration step | s |

The heading in YAML, the state display, and the Python API all use radians. State arrays have shape `(3, 1)` and velocity arrays have shape `(2, 1)`. The body-frame command is rotated into the world frame using the heading **at the beginning of the step**:

```{math}
\begin{aligned}
x_{k+1} &= x_k + v_k \cos(\theta_k)\,\Delta t,\\
y_{k+1} &= y_k + v_k \sin(\theta_k)\,\Delta t,\\
\theta_{k+1} &= \operatorname{wrap}_{[-\pi,\pi)}(\theta_k + \omega_k\,\Delta t).
\end{aligned}
```

This is the noise-free update in {py:func}`~irsim.lib.algorithm.kinematics.differential_kinematics`. It is a kinematic approximation, not a model of forces, friction, or wheel slip. For the underlying mobile-robot model, see Lynch and Park, *Modern Robotics: Mechanics, Planning, and Control* (2017), Chapter 13.

## Reproduce the default result in Python

This example calls the same integration function as the simulator, without rendering or a controller. It matches the lab's default inputs.

```python
import numpy as np
from irsim.lib.algorithm.kinematics import differential_kinematics

state = np.zeros((3, 1))
command = np.array([[0.8], [0.6]])  # m/s, rad/s
dt = 0.2
steps = 20  # four simulated seconds

for _ in range(steps):
    state = differential_kinematics(state, command, dt, noise=False)

np.testing.assert_allclose(state[:, 0], [1.03852807, 2.25970740, 2.4], atol=1e-8)
print(state[:, 0])
```

The browser experiment uses `env.step(action=[0.8, 0.6])` on a full `diff` environment. Its default velocity limits do not clip this command, so its result agrees with the low-level check above. Download the [default YAML](../_static/playground/kinematics.yaml) to reproduce the scene locally. See [Make Environment](../usage/make_environment.md) for the complete loop.

## What does the error measure?

For a constant command and nonzero yaw rate, the continuous-time solution is a circular arc:

```{math}
\begin{aligned}
x(t) &= x_0 + \frac{v}{\omega}\bigl[\sin(\theta_0+\omega t)-\sin(\theta_0)\bigr],\\
y(t) &= y_0 - \frac{v}{\omega}\bigl[\cos(\theta_0+\omega t)-\cos(\theta_0)\bigr].
\end{aligned}
```

At zero yaw rate the solution is a straight line. The Euclidean distance between the simulated position and this analytic position measures **numerical integration error for this model**, not real-world localization or sensing error. With the defaults at four seconds it is approximately `0.149156 m`. This comparison assumes a constant, unclipped command, no noise, and no collision-induced stopping.

For smooth motion over a fixed duration, forward Euler has first-order global error: reducing the step size reduces the leading integration error approximately in proportion. This example is not a collision-accuracy benchmark. Smaller steps do not turn sampled collision checks into continuous collision detection.

## Continue learning

- [Configure robots and obstacles](../usage/configure_robots_obstacles.md): choose a footprint, kinematics model, and velocity limits.
- [Configure sensors](../usage/configure_sensor.md): turn geometry into measurements.
- [Path planning](../usage/configure_path_planning.md): distinguish a geometric path from executable commands.
