# Quick Start

This example runs a differential-drive robot from a start pose to a goal. Before
continuing, [install IR-SIM](install.rst) and verify that it imports successfully.

1. Create a Python file named `quick_start.py`:

```python
import irsim

env = irsim.make("robot_world.yaml")

for _ in range(300):
    env.step()
    env.render()

    if env.done():
        break

env.end()
```

2. In the same directory, create `robot_world.yaml`:

The YAML file describes the world and robot. Change these values later to customize the scene.

```yaml
world:
  height: 10
  width: 10
  step_time: 0.1
  sample_time: 0.1
  offset: [0, 0]

robot:
  kinematics: {name: diff}
  shape: {name: circle, radius: 0.2}
  state: [1, 1, 0]
  goal: [9, 9, 0]
  behavior: {name: dash}
  color: g
  plot:
    show_trajectory: true
    show_goal: true
```

3. Run the Python file from that directory:

```bash
python quick_start.py
```

Run the script and a window opens showing the differential-drive robot navigating from its start to the goal:

```{image} https://raw.githubusercontent.com/IR-SIM/ir-sim-gifs/main/get_started/quick_start.gif
:alt: Quick-start simulation: a differential-drive robot navigating to its goal
:width: 420px
:align: center
```

The loop advances the simulation by `0.1` seconds per step, renders the latest
state, and stops early when the environment reports that it is done. For a
server or batch job, use `headless=True` to avoid creating a figure entirely;
`env.render()` then becomes a no-op. Use `display=False` instead if you still
need offscreen figures or animations.

## Try one change at a time

1. Change `goal` to `[9, 1, 0]`. The robot can now reach the goal along a straight line.
2. Set `step_time` to `0.05` and increase the loop budget to `600`. Both budgets represent up to 30 simulated seconds; compare the paths, not just the frame counts.
3. Explore the [kinematics lab](kinematics.md) to see how heading and velocity produce motion before adding obstacles or a controller.

The default `dash` behavior chooses commands toward the goal. It is not a path planner: adding an obstacle does not automatically make the robot plan around it. Continue with [behaviors](../usage/configure_behavior.md) or [path planning](../usage/configure_path_planning.md) for avoidance and planning.

## Next steps

- [Make Environment](../usage/make_environment.md): the simulation loop, status control, and dynamic objects in depth.
- [Configure robots and obstacles](../usage/configure_robots_obstacles.md): kinematics, shapes, and multi-object scenes.
- [YAML Configuration](../yaml_config/configuration.md): the full configuration reference with an interactive schema explorer.
