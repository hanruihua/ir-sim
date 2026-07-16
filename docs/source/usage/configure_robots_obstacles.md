# Configure robots and obstacles

To effectively simulate robots within your environment, you need to define and configure various robot parameters.

## Robot Configuration Parameters

Each robot in the simulation is defined by a set of parameters in a YAML configuration file. Below is a simple example of a robot configuration:

The python script and YAML configuration file:

::::{tab-set}

:::{tab-item} Python Script
```python
import irsim

env = irsim.make('robot_world.yaml')

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
- **`shape`:** Specifies the physical shape and size of the robot. Name options include `'circle'`, `'rectangle'`, `'polygon'`, and `linestring`.
    - `circle`: A circular robot with a specified radius.
    - `rectangle`: A rectangular robot with specified length and width.
    - `polygon`: A polygonal robot.
    - `linestring`: list of lines.
 
- **`state`:** Defines the initial position and orientation of the robot in the environment.
- **`goal`:** Specifies the target position and orientation for the robot.
- **`behavior`:** Specifies how the robot generates velocity in `env.step()` when no external command is provided. If omitted, the robot remains static unless a velocity command is passed to `env.step(velocity)`.
- **`plot`** (optional): Specifies the visualization settings for the robot. See {py:meth}`~irsim.world.object_base.ObjectBase.plot` for more details.


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

env = irsim.make('robot_world.yaml')

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

:::{note}
**Robot vs Obstacle - Key Differences:**

| Parameter | Robot Default | Obstacle Default |
|-----------|---------------|------------------|
| `role` | `"robot"` | `"obstacle"` |
| `color` | Varies | `"k"` (black) |
| `kinematics` | User-defined | `None` (static) |
| `behavior` | `None` (static unless configured or externally controlled) | `None` (static unless configured or externally controlled) |

**Configuration Tips:**
- Objects without `kinematics` are static
- Add `kinematics` + `behavior` to create moving robots or obstacles
- Pass a velocity to `env.step(velocity)` when using your own controller instead of a configured behavior
- Use `-` to define each new robot/obstacle in the list
:::

:::{warning}
Please make sure that the obstacles are not placed in the initial position of the robot. Otherwise, the robot will collide with the obstacles at the beginning of the simulation.
:::

## Advanced Configurations for Multiple Robots and Obstacles

To simulate multiple robots and obstacles within the same environment, simply add the `number` and `distribution` of robots and obstacles to the configuration file. Below is an example of a configuration file with multiple robots and obstacles: 

::::{tab-set}

:::{tab-item} Python Script

```python

import irsim

env = irsim.make('robot_world.yaml')

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
