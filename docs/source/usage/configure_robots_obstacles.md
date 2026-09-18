# Configure robots and obstacles

To effectively simulate robots within your environment, you need to define and configure various robot parameters.

## Robot Configuration Parameters

Each robot in the simulation is defined by a set of parameters in a YAML configuration file. Below is a simple example of a robot configuration:

The python script and YAML configuration file:

::::{tab-set}

:::{tab-item} Python Script
```python
import irsim

env = irsim.make("robot_world.yaml")

for i in range(1000):
    env.step()
    env.render(0.05)

    if env.done():
        break

env.end()
```
:::

:::{tab-item} YAML Configuration
```yaml
world:
  height: 10 
  width: 10   

robot:
  kinematics: {name: 'diff'}  
  shape: {name: 'circle', radius: 0.2}  
  state: [1, 1, 0]  
  goal: [9, 9, 0] 
  behavior: {name: 'dash'}
  color: 'g'
  plot:
    show_trajectory: True
    show_goal: True
```
:::

:::{tab-item} Demonstration

```{image} https://raw.githubusercontent.com/IR-SIM/ir-sim-gifs/main/get_started/quick_start.gif
:alt: Differential-drive robot moving from its start pose to its goal
:width: 400px
:align: center
```
:::

::::

### Important Parameters Explained

- **`kinematics`:** Defines the movement model of the robot. Name options include `'omni'`, `'omni_angular'`, `'diff'`, and `'acker'`. 
  - `'omni'`: Omnidirectional wheels allowing movement in all directions (no orientation control).
  - `'omni_angular'`: Omnidirectional with yaw rate control (can translate and rotate independently).
  - `'diff'`: Differential drive allowing movement forward/backward and rotation.
  - `'acker'`: Ackermann steering, typical for car-like robots.

**Kinematics Comparison:**

| Kinematics | Control Input | Typical Use | Can Rotate in Place? |
|------------|---------------|-------------|----------------------|
| `omni`     | `[forward, lateral]` - body-frame velocity | Holonomic robots, drones | ✗ No |
| `omni_angular` | `[forward, lateral, yaw_rate]` - body-frame velocity + yaw | Holonomic robots with rotation | ✓ Yes |
| `diff`     | `[v, ω]` - linear & angular velocity | Two-wheeled robots | ✓ Yes |
| `acker`    | `[v, φ]` - linear velocity & steering angle | Cars, car-like robots | ✗ No |
- **`shape`:** Specifies the physical shape and size of the robot. Name options include `'circle'`, `'rectangle'`, `'polygon'`, `'compound'`, and `linestring`.
    - `circle`: A circular robot with a specified radius.
    - `rectangle`: A rectangular robot with specified length and width.
    - `polygon`: A polygonal robot.
    - `compound`: A rigid combination of circle, rectangle, or polygon parts.
    - `linestring`: list of lines.
 
- **`state`:** Defines the initial position and orientation of the robot in the environment.
- **`goal`:** Specifies the target position and orientation for the robot.
- **`behavior`:** Specifies how the robot generates velocity in `env.step()` when no external command is provided. If omitted, the robot remains static unless a velocity command is passed to `env.step(velocity)`.
- **`plot`** (optional): Specifies the visualization settings for the robot. See {py:meth}`~irsim.world.object_base.ObjectBase.plot` for more details.

For a `compound` shape, the object's `state` moves the complete body. Each part's optional `pose: [x, y, theta]` is fixed relative to the object frame and defaults to `[0, 0, 0]`. All parts use the owning object's color. `show_trajectory` uses the same path-line renderer as other shapes. By default, its width is the compound's local-y extent and the line follows the midpoint of that extent, keeping the trajectory and body widths aligned. The line does not reproduce concave or disjoint footprint gaps; use `show_trail` for exact historical shape snapshots.


The example above explicitly sets `behavior: {name: 'dash'}` so the robot moves from its initial state toward its goal when `env.step()` is called without an input velocity.

### Explanation

- **`env.step()`:** Advances the simulation by one time step. You can input your control commands here by `env.step(velocity)` to run your own control algorithm. `velocity` is associated with the `kinematics` of the robot. See {py:meth}`~irsim.env.env_base.EnvBase.step` for more details. 
- **`env.render(0.05)`:** Renders the current state of the environment with a 0.05-second delay between frames. See {py:meth}`~irsim.env.env_base.EnvBase.render` for more details.
- **`env.done()`:** Checks whether the simulation conditions to terminate have been met. Such as reaching the goal or a collision. See {py:meth}`~irsim.env.env_base.EnvBase.done` for more details.
- **`env.end()`:** Ensures that the simulation is terminated gracefully, releasing any resources or handles. Provides a clean exit. See {py:meth}`~irsim.env.env_base.EnvBase.end` for more details.

:::{note}
The [rda_planner](https://github.com/hanruihua/RDA-planner) is a case of using the `env.step(velocity)` to run your own control algorithm.
:::

:::{note}
You can add Gaussian noise on the kinematics of the robot and obstacle by setting the `noise` to be True in parameter in the `kinematics` dictionary. See the {doc}`YAML configuration <../yaml_config/configuration>` for more details.
:::

## Obstacle Configuration Parameters

The parameters of obstacles in the simulation are similar to those of robots. Below is an example of adding various obstacles to the yaml configuration file, and run the same python script as above.

::::{tab-set}

:::{tab-item} Python Script

```python
import irsim

env = irsim.make("robot_world.yaml")

for i in range(1000):
    env.step()
    env.render(0.05)

    if env.done():
        break

env.end()
```
:::

:::{tab-item} YAML Configuration

```yaml
world:
  height: 10 
  width: 10   

robot:
  kinematics: {name: 'diff'}  
  shape: {name: 'circle', radius: 0.2}  
  state: [1, 1, 0]  
  goal: [9, 9, 0] 
  behavior: {name: 'dash'}
  color: 'g'
  plot:
    show_trajectory: True
    show_goal: True

obstacle:
  - shape: {name: 'circle', radius: 1.0}  # radius
    state: [5, 5, 0]  
  
  - shape: {name: 'rectangle', length: 1.5, width: 1.2}  # length, width
    state: [6, 5, 1] 

  - shape: {name: 'linestring', vertices: [[5, 5], [4, 0], [1, 6]] }  # vertices
    state: [0, 0, 0] 
    unobstructed: True

  - shape:
      name: 'polygon'
      vertices: 
        - [4.5, 4.5]
        - [5.5, 4.5]
        - [5.5, 5.5]
        - [4.5, 5.5]
```
:::

:::{tab-item} Demonstration
:selected:

```{image} https://raw.githubusercontent.com/IR-SIM/IR-SIM-GIFs/main/robots_obstacles/robot_obstacle.gif
:alt: Select Parameters
:width: 400px
:align: center
```
:::
::::


### Important Parameters Explained

- **unobstructed**: If `True`, there is no collision detection with the object. 
- **mass**: Mass in kilograms. Only used by `collision_mode: 'contact'`, where a finite mass makes the obstacle a pushable body and no mass leaves it immovable. Pushable obstacles are drawn in orange (`#E69F00`) by default. See [Physical properties and contact mode](#physical-properties-and-contact-mode).

:::{note}
**Robot vs Obstacle - Key Differences:**

| Parameter | Robot Default | Obstacle Default |
|-----------|---------------|------------------|
| `role` | `"robot"` | `"obstacle"` |
| `color` | Varies | `"k"` (black) |
| `kinematics` | User-defined | `None` (static) |
| `behavior` | `None` (static unless configured or externally controlled) | `None` (static unless configured or externally controlled) |

**Configuration Tips:**
- Objects without `kinematics` are static, unless they are given a finite `mass` for the `contact` collision mode
- Add `kinematics` + `behavior` to create moving robots or obstacles
- Pass a velocity to `env.step(velocity)` when using your own controller instead of a configured behavior
- Use `-` to define each new robot/obstacle in the list
:::

:::{warning}
Please make sure that the obstacles are not placed in the initial position of the robot. Otherwise, the robot will collide with the obstacles at the beginning of the simulation.
:::

## Physical Properties and Contact Mode

By default (`collision_mode: 'stop'`) objects halt when they touch. Setting `collision_mode: 'contact'` in the `world` section turns collisions into contacts: after every step, overlapping objects are pushed apart along their contact normal, and the separation is shared in inverse proportion to their `mass`. This is enough for a robot to push a box, for a heavy box to slow the robot down, and for a wall to stop it while it slides along the surface.

- **`mass`** (`float`, kg): the only physical property an object needs. Objects with kinematics default to `1.0`. Objects without kinematics have no mass (`inf`) and never move; give one a finite `mass` and it becomes a dynamic body that can be pushed but does not move on its own. A list under `number` sets one mass per object. A pushable obstacle is drawn in orange (`#E69F00`) unless you set `color`, so you can tell at a glance what moves; the shade is the colour-blind-safe Okabe-Ito orange and stays distinct from black when printed in grayscale. Static objects and the grid map are immovable regardless.
- **What the contact does**: with inverse masses `w = 1 / mass`, object A moves by `depth * w_A / (w_A + w_B)` and B by the rest. Two equal masses split the overlap evenly, so a robot commanding 1 m/s pushes an equal box at 0.5 m/s; a 4 kg box is pushed at 0.2 m/s by a 1 kg robot; a box against a wall stops the robot. The pushed displacement is folded into each object's velocity, so lidar, RVO and SFM neighbors, and `env.get_msg()` see the box moving.
- **What it does not do**: bodies only translate, so a box pushed off-center does not spin, and there is no inertia or friction: an object stops as soon as nothing pushes it (quasi-static pushing). Objects are never stopped by contacts, so `collision` stays `False` and `env.done()` only reports arrival; `obj.contact` and `obj.contact_obj` tell which objects touched in the last step. Contacts are skipped in `step_mode: 'external'`, and keep `speed * step_time` below an object's radius so it cannot tunnel through a thin `linestring` wall in one step.

The example below (`usage/26push_box_world/`) has three identical robots push boxes of 0.25, 1 and 4 kg toward a wall:

::::{tab-set}

:::{tab-item} Python Script

```python
import irsim

env = irsim.make("push_box_world.yaml")
boxes = env.obstacle_list[:3]

for _ in range(200):
    env.step()
    env.render(0.02)

for robot, box in zip(env.robot_list, boxes, strict=True):
    print(f"{box.name} ({box.mass:g} kg) moved {box.state[0, 0] - 3:.2f} m")

env.end(3)
```
:::

:::{tab-item} YAML Configuration

```yaml
world:
  height: 12
  width: 12
  step_time: 0.1
  collision_mode: 'contact'   # touching objects push each other apart by mass

robot:
  - number: 3
    distribution: {name: 'manual'}
    kinematics: {name: 'diff'}
    shape: [{name: 'circle', radius: 0.3}]
    mass: 1.0
    state: [[1, 2, 0], [1, 6, 0], [1, 10, 0]]
    goal: [[11.5, 2, 0], [11.5, 6, 0], [11.5, 10, 0]]
    behavior: {name: 'dash'}
    vel_max: [1.0, 1.0]

obstacle:
  - number: 3
    distribution: {name: 'manual'}
    shape: [{name: 'rectangle', length: 0.8, width: 0.8}]
    state: [[3, 2, 0], [3, 6, 0], [3, 10, 0]]
    mass: [0.25, 1.0, 4.0]      # a finite mass makes an obstacle pushable (drawn in orange)

  - shape: {name: 'rectangle', length: 0.4, width: 11}   # no mass: immovable wall
    state: [10.5, 6, 0]
```
:::

::::

The light box travels at almost the robot's speed, the equal one at half speed, and the heavy one barely moves; once a box reaches the wall the whole chain stops although the robots keep asking for full speed. `push_box_keyboard.yaml` in the same folder lets you shove boxes around with the keyboard.

:::{note}
Contacts are resolved by the separating axis theorem between convex pieces of the two shapes (circles, convex polygons, and line segments; non-convex polygons are triangulated), so every shape takes part: `compound` bodies, `linestring` walls, and grid maps. See {py:mod}`irsim.lib.algorithm.contact`.
:::

## Advanced Configurations for Multiple Robots and Obstacles

To simulate multiple robots and obstacles within the same environment, simply add the `number` and `distribution` of robots and obstacles to the configuration file. Below is an example of a configuration file with multiple robots and obstacles: 

::::{tab-set}

:::{tab-item} Python Script

```python
import irsim

env = irsim.make("robot_world.yaml")

for i in range(1000):
    env.step()
    env.render(0.05)

    if env.done():
        break

env.end()
```
:::

:::{tab-item} YAML Configuration

```yaml
world:
  height: 10  # the height of the world
  width: 10   # the width of the world

robot:
  - number: 2
    distribution: {name: 'manual'}
    kinematics: {name: 'diff'}
    shape: 
      - {name: 'circle', radius: 0.2}  # radius
    state: 
      - [1, 1, 0]  
      - [2, 1, 0]
    goal:
      - [9, 9, 0] 
      - [9, 2, 0]
    behavior:
      - {name: 'dash'}
      - {name: 'dash'}
    color: 
      - 'royalblue'
      - 'red'
  
  - number: 4
    distribution: {name: 'random'}
    kinematics: {name: 'diff'}
    shape: 
      - {name: 'circle', radius: 0.2}  # radius
    color: 
      - 'pink'


obstacle:
  - number: 4
    distribution: {name: 'manual'}
    state: [[4, 8], [1, 3], [1, 0], [5, 2]]
    shape:
      - {name: 'circle', radius: 0.2}  # radius
      - {name: 'circle', radius: 0.1}  # radius
    color: 'k'
```

:::{note}
- The `distribution` parameter specifies how the robots and obstacles are distributed within the environment. Options include `'manual'` and `'random'`. Details are provided in the {doc}`YAML Configuration <../yaml_config/configuration>`.
:::

:::{tab-item} Demonstration
:selected:

```{image} https://raw.githubusercontent.com/IR-SIM/IR-SIM-GIFs/main/robots_obstacles/multi_objects.gif
:alt: Select Parameters
:width: 400px
:align: center
```
:::
::::

## Advanced Configuration for Custom Kinematics

If the built-in models (`diff`, `omni`, `omni_angular`, `acker`) do not match your robot, register your own kinematics handler. Subclass `KinematicsHandler` (or one of the built-in handlers), add any parameters your model needs to `__init__`, and register it with `@register_kinematics`:

```python
import numpy as np
from irsim.lib import register_kinematics
from irsim.lib.handler.kinematics_handler import DifferentialKinematics


@register_kinematics("lag_diff")
class LagDiffKinematics(DifferentialKinematics):
    """Differential drive whose velocity follows the command with a first-order lag."""

    def __init__(self, name, noise=False, alpha=None, tau=0.5):
        super().__init__(name, noise, alpha)
        self.tau = tau  # time constant of the lag, in seconds
        self._vel = None

    def step(self, state, velocity, step_time):
        if self._vel is None:
            self._vel = np.zeros_like(velocity)
        self._vel = self._vel + (velocity - self._vel) * min(step_time / self.tau, 1.0)
        return super().step(state, self._vel, step_time)
```

Any key under `kinematics` other than `name`, `noise`, and `alpha` is passed to the handler's `__init__`, so the model's parameters are set directly in YAML:

```yaml
robot:
  - kinematics: {name: 'lag_diff', tau: 0.5}
    shape: {name: 'circle', radius: 0.2}
    state: [1, 1, 0]
```

:::{important}
The module that defines the handler must be imported before `irsim.make()` so the registration runs. An unknown `name` raises `NotImplementedError`, and a key the handler does not accept raises `TypeError`, so typos in either are reported immediately.
:::
