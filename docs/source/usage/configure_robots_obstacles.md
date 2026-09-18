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
- **mass**: Mass in kilograms. Only used by `collision_mode: 'contact'`, where a finite mass makes the obstacle a pushable body and no mass leaves it immovable. Pushable obstacles without kinematics are drawn in orange (`#E69F00`) by default. See [Physical properties and contact mode](#physical-properties-and-contact-mode).
- **friction**: Coulomb friction coefficient with the ground, `0.5` by default. Only used by `collision_mode: 'contact'`, where it slows a released body to a stop and, with `mass`, decides whether a robot can push it. See [Physical properties and contact mode](#physical-properties-and-contact-mode).
- **inertia**: Moment of inertia in kg·m², computed from the shape and mass unless set. Only used by `collision_mode: 'contact'`, where a push that misses a pushable obstacle's center turns it. See [Physical properties and contact mode](#physical-properties-and-contact-mode).
- **restitution**: Bounciness of the material from `0` (default, no bounce) to `1`. Only used by `collision_mode: 'contact'`. See [Physical properties and contact mode](#physical-properties-and-contact-mode).

:::{note}
**Robot vs Obstacle - Key Differences:**

| Parameter | Robot Default | Obstacle Default |
|-----------|---------------|------------------|
| `role` | `"robot"` | `"obstacle"` |
| `color` | green `#009E73` (`#117733` for `acker`) | `"k"` (black) |
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

By default (`collision_mode: 'stop'`) objects halt when they touch. Setting `collision_mode: 'contact'` in the `world` section turns collisions into contacts: after every step, overlapping objects are pushed apart along their contact normal, following the rules a physics engine such as PhysX (Isaac Sim, CARLA) applies to rigid bodies, reduced to `mass`, `friction`, `inertia` and `restitution`. A robot pushes a box at its own speed as long as its traction beats the box's ground friction, a box too heavy for it stalls it, its wheels' grip also bounds how fast it can launch or stop, a box hit off-center turns, a released box slides to a stop, a wall stops whatever presses on it, and the robot's drive can be given the lag of a real base.

- **`mass`** (`float`, kg): Objects with kinematics default to `1.0`. Objects without kinematics have no mass (`inf`) and never move; give one a finite `mass` and it becomes a pushable body that does not move on its own. A list under `number` sets one mass per object. Such a body is drawn in orange (`#E69F00`) unless you set `color`, so you can tell at a glance what moves only when pushed, while obstacles with kinematics keep their usual color; the shade is the colour-blind-safe Okabe-Ito orange and stays distinct from black when printed in grayscale. With `friction` the mass gives an object's ground friction force, `friction * mass * g`: a robot pushes a body while its own value is at least that of the body plus everything the body pushes ahead of it, so with equal materials it pushes up to its own mass in boxes, in one or in a row, and stalls against more. Static objects and the grid map are immovable regardless.
- **`friction`** (`float`, default: the world's `friction`, `0.5`): Coulomb friction coefficient with the ground, the default material value of PhysX. A released body decelerates by `friction * gravity`, 9.81 m/s² by default, until it stops, so from 1 m/s it slides about 5 cm, and `friction: 0` lets it slide forever. A body pressed against a wall slides along it only when the push leaves the friction cone of the two surfaces (their mean coefficient; at 0.5, pushes more than 27° from the wall normal slide), while a robot always slides, since its wheels can drive along the wall. A list under `number` sets one coefficient per object.
- **`inertia`** (`float`, kg·m²): moment of inertia about the object's center of mass (its centroid), computed from its shape and mass unless set: `m r² / 2` for a disc, `m (l² + w²) / 12` for a rectangle, the exact polar moment for a polygon. A box is turned by any off-center push; a robot keeps its heading while it pushes, but stopped by a wall or a load it cannot move, an off-center contact deflects it as slipping wheels would. A spinning body slows under friction like a sliding one.
- **`restitution`** (`float`, default: the world's `restitution`, `0`): bounciness of the material, `0` for a perfectly inelastic contact as in Isaac Lab's default material, `1` for an elastic one. Two bodies that collide separate at the mean of their values times their approach speed: a frictionless disc with `restitution: 1` pushed at 1 m/s by a robot of the same material leaves it at 2 m/s and comes back off a wall of that material at the speed it arrived, while the default `0` on the robot or the wall halves the bounce. Only passive bodies bounce, and only off a contact that could move them: a box a robot cannot push is not hammered forward by repeated kicks. A robot's drive re-asserts its velocity.
- **Traction limit**: in contact mode a robot's wheels can only change its speed by `friction * gravity` per second, since that is all the grip they have on the floor. With the default `0.5` a launch or a stop from 1 m/s takes three 0.1 s steps and about 5 cm, so a controller cannot stop or reverse a robot within a step, and a robot with `friction: 0` cannot move at all, as in a physics engine. Set `friction: 1.0` on a robot for rubber wheels that both push and accelerate harder.
- **Drive lag** (`kinematics: {name: 'diff', tau: 0.2}`): in contact mode a robot's actual velocity can follow its command as a first-order response with time constant `tau`, as a base with a soft velocity loop does, so it cannot stop or reverse within a step. The default is `0`, instant tracking, which matches Isaac's stiff default drives at this step size and which the other collision modes always use. A stalled robot keeps pushing at its full command while its body velocity reads zero, like slipping wheels.
- **World defaults**: `gravity` (`9.81`), `friction` (`0.5`), `restitution` (`0`) and `drive_tau` (`0`) under the `world` section are the physics every object starts from; a per-object `friction` or `restitution`, or a `tau` under its `kinematics`, overrides them. Lower the world `friction` for a slippery floor, or set `drive_tau: 0.2` to give every robot the drive lag of a small base.
- **What the contact does**: a pushed body moves with its pusher, so a 1 kg robot commanding 1 m/s pushes a 0.25 kg or a 1 kg box at 1 m/s, while a 4 kg box stalls it, since the robot's traction `0.5 * 1 kg * g` is below the box's ground friction `0.5 * 4 kg * g`; a box against a wall stops the robot. Between two passive bodies, or two robots, the overlap is split in inverse proportion to `mass`, which is a perfectly inelastic collision: a 1 kg box sliding at 1 m/s into a resting 3 kg box leaves both at 0.25 m/s. A push that misses a body's center also turns it: the contact point and normal give a torque that the body's moment of inertia resists, so a box hit near a corner spins as it is pushed and a box pressed into a wall by one corner pivots around it, while a push through the center only slides it. The pushed displacement and rotation are folded into each object's velocity, so lidar, RVO and SFM neighbors, and `env.get_msg()` see the box moving.
- **What it does not do**: a robot that pushes successfully is never turned, since its drive holds its heading, and nothing bounces unless a `restitution` is set. Objects are never stopped by contacts, so `collision` stays `False` and `env.done()` only reports arrival; `obj.contact`, `obj.contact_obj` and `obj.contact_force` (newtons, world frame, estimated from the constraint impulse as XPBD does) tell which objects touched in the last step and how hard: a box pushed steadily reports the ground friction it overcomes, a stalled robot its traction, the most its slipping wheels can push with. Contacts are skipped in `step_mode: 'external'`, and keep `speed * step_time` below an object's radius so it cannot tunnel through a thin `linestring` wall in one step.

The example below (`usage/26push_box_world/`) has three identical robots push boxes of 0.25, 2 and 1 kg toward a wall:

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
    mass: [0.25, 2.0, 1.0]      # a finite mass makes an obstacle pushable (drawn in orange)

  - shape: {name: 'rectangle', length: 0.4, width: 11}   # no mass: immovable wall
    state: [10.5, 6, 0]
```
:::

::::

The 0.25 kg and the 1 kg box travel at the robot's speed, while the 2 kg box is too heavy for a 1 kg robot to push and stalls it; once a box reaches the wall the whole chain stops although the robots keep asking for full speed. `push_box_keyboard.yaml` in the same folder lets you shove boxes around with the keyboard.

:::{note}
Contacts are resolved by the separating axis theorem between convex pieces of the two shapes (circles, convex polygons, and line segments; non-convex polygons are triangulated), so every shape takes part: `compound` bodies, `linestring` walls, and grid maps. See {py:mod}`irsim.lib.algorithm.contact`.
:::

## Advanced Configurations for Multiple Robots and Obstacles

To simulate multiple robots and obstacles within the same environment, simply add the `number` and `distribution` of robots and obstacles to the configuration file. Below is an example of a configuration file with multiple robots and obstacles: 

Robots in a group share the default robot color. Set `color: 'cycle'` on the group to give each robot the next color of the palette cycle instead, which helps telling them apart in a crowd; a list of colors still assigns them one by one. The defaults themselves are parameters of {py:mod}`irsim.config.palette_param` and can be changed before `irsim.make()`.

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
    color: 'cycle'   # one palette color per robot, in order
  
  - number: 4
    distribution: {name: 'random'}
    kinematics: {name: 'diff'}
    shape: 
      - {name: 'circle', radius: 0.2}  # radius


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


@register_kinematics("slip_diff")
class SlipDiffKinematics(DifferentialKinematics):
    """Differential drive whose wheels slip: only part of the commanded speed reaches the ground."""

    def __init__(self, name, noise=False, alpha=None, ratio=0.8):
        super().__init__(name, noise, alpha)
        self.ratio = ratio  # fraction of the commanded linear speed that is achieved

    def step(self, state, velocity, step_time):
        achieved = np.array(velocity, dtype=float)
        achieved[0, 0] *= self.ratio
        return super().step(state, achieved, step_time)
```

Any key under `kinematics` other than `name`, `noise`, `alpha`, and the built-in drive lag `tau` is passed to the handler's `__init__`, so the model's parameters are set directly in YAML:

```yaml
robot:
  - kinematics: {name: 'slip_diff', ratio: 0.8}
    shape: {name: 'circle', radius: 0.2}
    state: [1, 1, 0]
```

:::{important}
The module that defines the handler must be imported before `irsim.make()` so the registration runs. An unknown `name` raises `NotImplementedError`, and a key the handler does not accept raises `TypeError`, so typos in either are reported immediately.
:::
