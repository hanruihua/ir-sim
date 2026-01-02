# YAML Configuration Syntax

The configuration file is a YAML file to initialize the environment. It contains the configuration parameters to simulate and visualize the world, obstacle, and robot. You can customize the scenario and define the behavior of the objects simply by using the parameters.

---

## Parameter Quick Reference

Use this navigation to quickly jump to specific parameter sections:

::::{dropdown} **World Parameters**
:color: primary
:icon: globe
:open:
- [Parameters Table](#world-parameters-table)
- [world properties](#world-properties)
  - `name`, `height`, `width`, `step_time`, `sample_time`, `offset`
- [world mode](#world-mode)
  - `control_mode`, `collision_mode`, `status`
- [world map](#world-map)
  - `obstacle_map`, `mdownsample`
- [world visualization](#world-visualization)
  - `plot`
::::

::::{dropdown} **Object Parameters**
:color: info
:icon: server
:open:
- [Parameters Table](#object-parameters-table)
- [object properties](#object-properties)
  - `number`, `distribution`, `state`, `goal`, `velocity`, `state_dim`, `vel_dim`, `group`, `name`
- [object kinematics](#object-kinematics)
  - `kinematics`, `vel_min`, `vel_max`, `acce`, `angle_range`, `goal_threshold`
- [object shape](#object-shape)
  - `shape`
- [object behavior](#object-behavior)
  - `behavior`, `group_behavior`, `role`, `static`,
- [object sensors](#object-sensor)
  - `sensors`, `fov`, `fov_radius`
- [object mode](#object-mode)
  - `arrive_mode`, `unobstructed`
- [object visualization](#object-visualization)
  - `color`, `plot`, `description`
::::

::::{dropdown} **GUI Parameters**
:color: warning
:icon: key
:open:
- [Parameters Table](#gui-parameters-table)
- [keyboard properties](#keyboard-properties)
  - `backend`, `global_hook`, `key_lv_max`, `key_ang_max`, `key_lv`, `key_ang`, `key_id`
- [mouse properties](#mouse-properties)
  - `zoom_factor`
::::

---

**Quick Start Example**

The configuration file is divided into three main sections: `world`, `robot`, and `obstacle`. Here's a simple example:

::::{tab-set}

:::{tab-item} Basic Configuration
```yaml
world:
  height: 10  # the height of the world
  width: 10   # the width of the world
  step_time: 0.1  # 10Hz calculate each step
  sample_time: 0.1  # 10 Hz for render and data extraction 
  offset: [0, 0] # the offset of the world on x and y 
  collision_mode: 'stop'  # 'stop', 'unobstructed', 'unobstructed_obstacles'
  plot:
    show_title: true
    figure_pixels: [1000, 800]

robot:
  - kinematics: {name: 'diff'}  # omni, diff, acker
    shape: {name: 'circle', radius: 0.2}  # radius
    # shape: {name: 'rectangle', length: 0.5, width: 0.2}  # radius
    state: [1, 1, 0]  
    goal: [9, 9, 0] 
    # acce: [3, .inf]   # acce of [linear, angular]  or [v_x, v_y] or [linear, steer]
    behavior: {name: 'dash'} # move toward to the goal directly 
  
obstacle:
  - number: 10
    distribution: {name: 'random'}
    shape: 
      - {name: 'circle', radius: 1.0}  # radius
      - {name: 'rectangle', length: 1.5, width: 1.2}  # radius
    state: 
      - [5, 5, 0]  
      - [4, 4, 0]
  
  - shape: {name: 'rectangle', length: 1.5, width: 1.2}  # radius
    state: [6, 5, 1] 

  - shape: {name: 'linestring', vertices: [[5, 5], [4, 0], [1, 6]] }  # vertices
    state: [0, 0, 0] 
    unobstructed: True
```
:::

::::

````{important}
**Key Configuration Guidelines:**

- To include several robots or obstacles in the configuration file, add separate entries under the robot and obstacle sections using `-` for each additional item.
- Parameters such as distribution, shape, behavior, group_behavior, and kinematics must be formatted as `{key: value}` pairs. Ensure that each dictionary includes the `name` key; omitting name will result in a None value for that parameter.
- When dealing with multiple objects (i.e., when the number is greater than 1), utilize the `distribution` parameter to define how these objects are distributed.
- By default, all objects within the same group share identical configurations. To customize individual objects within a group, add sub-parameters using `-`. Any additional objects not explicitly configured will inherit the settings of the last specified object in the group.
- Object-level `name` is optional and identifies each object. If omitted, it defaults to `"<role>_<id>"` (e.g., `robot_0`, `obstacle_3`). Names must be unique across all objects; duplicates raise an error during environment initialization. Do not confuse object-level `name` with the `name` keys used inside dictionaries like `shape`, `kinematics`, or `behavior` (which indicate component types).
````

---

## World Configuration

This section outlines the configuration parameters available for the `world` section.  

(world-parameters-table)=
### World Parameters Table

| **Parameter**    | **Type**          | **Default** | **Description**                                                                                            |
| ---------------- | ----------------- | ----------- | ---------------------------------------------------------------------------------------------------------- |
| `name`           | `str`             | `"world"`   | Name of the world                                                                                          |
| `height`         | `float`           | `10`        | Height of the world (meter)                                                                                |
| `width`          | `float`           | `10`        | Width of the world (meter)                                                                                 |
| `step_time`      | `float`           | `0.1`       | Time interval between simulation steps (in seconds)                                                        |
| `sample_time`    | `float`           | `step_time` | Time interval between samples for rendering and data extraction (in seconds). Defaults to `step_time` if not specified. |
| `offset`         | `list` of `float` | `[0, 0]`    | Offset for the world's position in `[x, y]` coordinates                                                    |
| `control_mode`   | `str`             | `"auto"`    | Control mode of the simulation. Support mode: `auto` or `keyboard`                                         |
| `collision_mode` | `str`             | `"stop"`    | Collision handling mode (Support: `"stop"`, `"unobstructed"`, `"unobstructed_obstacles"`)                  |
| `status`         | `str`             | `"None"`    | Initial status of the simulation environment (Support: `"Running"`, `"Arrived"`, `"Collision"`, `"Pause"`) |
| `obstacle_map`   | `str` (file path) | `None`      | Path to the image file representing the obstacle map                                                       |
| `mdownsample`    | `int`             | `1`         | Downsampling factor for the obstacle map to reduce resolution and decrease computational load.             |
| `plot`           | `dict`            | `{}`        | Plotting options for initializing the plot of the world.                                                   |

### Detailed Parameter Descriptions

(world-properties)=
::::{dropdown} **world properties**

**`name`** (`str`, default: `"world"`)
: Defines the name of the world used in the simulation. This can be useful for identifying different simulation environments.

**`height`** (`float`, default: `10`)
: Specifies the vertical size of the world in units of meters in the Y-axis direction plotted on the screen.

**`width`** (`float`, default: `10`)  
: Specifies the horizontal size of the world in units of meters in the X-axis direction plotted on the screen.

**`step_time`** (`float`, default: `0.1`)
: Determines the time interval between each simulation step. 
  
  **Performance Impact**: A smaller `step_time` results in a higher simulation frequency (e.g., `0.1` seconds corresponds to 10 Hz) but needs longer time to run the simulation.

**`sample_time`** (`float`, default: `step_time`)
: Defines the time interval for rendering the simulation and extracting data. This controls how frequently visual updates and data recordings occur. If not specified, defaults to the value of `step_time`.

**`offset`** (`list` of `float`, default: `[0, 0]`)
: Sets the initial positional offset of the world on the X and Y axes. This is useful for positioning the world within a larger coordinate system or for relative placement.
::::

(world-mode)=
::::{dropdown} **world mode**

**`control_mode`** (`str`, default: `"auto"`)
: Configures how the objects in the simulation are controlled:

  **Options:**
  - `auto`: Automatic control by the input velocities defined in python script or behavior in the YAML file.
  - `keyboard`: Manual control via keyboard inputs. The key inputs are defined in the file.

**`collision_mode`** (`str`, default: `"stop"`)
: Defines how collisions between objects are handled in the simulation:

  **Options:**
  - `stop`: Stops the movement of objects upon collision. (default)
  - `unobstructed`: Allows objects to pass through each other without consideration of any collision.
  - `unobstructed_obstacles`: Only allows obstacles to pass through each other without consideration of any collision. The robots will stop when they are in collision with the obstacles.

**`status`** (`str`, default: `"None"`)
: Sets the initial status of the simulation environment:

  **Options:**
  - `"Running"`: The simulation runs normally (default).
  - `"Pause"`: The simulation starts in a paused state.
  - `"Arrived"`: The simulation stops when the robot arrives at the goal.
  - `"Collision"`: The simulation stops when the robot collides with an obstacle.
  
  **Note**: The status can be dynamically changed during simulation using keyboard controls (space key) or programmatically.
::::

(world-map)=
::::{dropdown} **world map**

**`obstacle_map`** (`str` (file path), default: `None`)
: Specifies the file path to an image that serves as the obstacle map. This image is used to generate the grid map that defines the positions of obstacles within the world. Each pixel in the image corresponds to a grid cell in the map, where the color of the pixel determines the presence of an obstacle.

  **Available Maps**: We provide some example maps in the `irsim/world/map` folder and you can also use your own map by 3D datasets like [HM3D](https://aihabitat.org/datasets/hm3d/), [MatterPort3D](https://niessner.github.io/Matterport/), [Gibson](http://gibsonenv.stanford.edu/database/), etc. See [here](https://github.com/hanruihua/ir-sim/tree/features/irsim/world/map/binary_map_generator_hm3d) for more details.

  ```yaml
  # Example usage
  obstacle_map: 'hm3d_2.png' # hm3d_1.png, hm3d_2.png, hm3d_3.png, hm3d_4.png, hm3d_5.png, hm3d_6.png, hm3d_7.png, hm3d_8.png, hm3d_9.png, cave.png
  ```

**`mdownsample`** (`int`, default: `1`)
: Sets the downsampling factor for the obstacle map image. 

  **Performance Tip**: A higher value reduces the resolution of the obstacle map, which can optimize the simulation performance by decreasing computational load. 
::::

(world-visualization)=
::::{dropdown} **world visualization**

**`plot`** (`dict`, default: `{}`)
: Specifies the plotting options for initializing the plot of the world.

  **Visualization Options:**
  - `saved_figure`: Default `dpi` is `100`; default format is `png`; default bbox_inches is `tight`. See [matplotlib.pyplot.savefig](https://matplotlib.org/stable/api/_as_gen/matplotlib.pyplot.savefig.html) for more details.
  - `figure_pixels`: Width and height of the figure in pixels. Default is `[1000, 800]`.
  - `show_title`: Whether to show the plot title. Default is `True`.
  - `title`: Custom title for the plot. If not specified, shows simulation time and status.
  - `no_axis`: Whether to show the axis. Default is `False`.
  - `tight`: Whether to use tight layout. Default is `True`.
  - `viewpoint`: Controls the center of the camera/viewport. Default is `null`.
    - `null`: Use the initial world bounds (no automatic panning).
    - `[x, y]`: Keep the viewport centered at the fixed point `[x, y]`.
    - `"<object_name>"`: Follow the object with that name (dynamic panning), e.g., `"robot_0"`.

  ```yaml
  # Examples
  plot:
    viewpoint: [3, 3]        # fixed center at (3, 3)
  # or
  plot:
    viewpoint: "robot_0"     # follow the object named robot_0
  ```
::::

### Complete World Configuration Example

```yaml
world:
  name: "world"                       # Name of the world
  height: 10                          # Height of the world
  width: 10                           # Width of the world
  step_time: 0.1                      # Time interval between steps (10 Hz)
  sample_time: 0.1                    # Time interval for rendering and data extraction (10 Hz)
  offset: [0, 0]                      # Positional offset of the world on the x and y axes
  control_mode: 'keyboard'            # Control mode ('auto' or 'keyboard')
  collision_mode: 'stop'              # Collision handling mode ('stop', 'unobstructed', 'unobstructed_obstacles')
  obstacle_map: "path/to/map.png"     # Path to the obstacle map image file
  mdownsample: 2                      # Downsampling factor for the obstacle map
  status: "Running"                   # Initial simulation status
  plot:                               # Plotting configuration
    show_title: true                  # Show plot title
    title: "Custom Simulation Title"  # Custom title (optional)
    figure_pixels: [1200, 800]        # Figure size in pixels
    viewpoint: "robot_0"              # Camera center: fixed [x, y], object name, or null
    saved_figure:                     # Figure saving options
      dpi: 150                        # Resolution for saved figures
      format: "png"                   # File format
```

````{warning}
**`obstacle_map`**: Replace `"path/to/map.png"` with the actual file path to your obstacle map image. Ensure that the image is in a compatible format (e.g., PNG, JPEG) and properly represents obstacle locations.
````

---

(object-configuration)=
## Object Configuration

All `robot` and `obstacle` entities in the simulation are configured as objects with similar parameters but may have different default values. This section outlines the configuration parameters available for these objects.

(object-parameters-table)=
### Object Parameters Table

| Parameter        | Type                                             | Default          | Description                                                                                                        |
| ---------------- | ------------------------------------------------ | ---------------- | ------------------------------------------------------------------------------------------------------------------ |
| `name`           | `str` or `list` of `str`                         | `None`           | Unique identifier for the object. If omitted, auto-assigned as `"<role>_<id>"`. Supports a list when `number > 1`. |
| `number`         | `int`                                            | `1`              | Number of objects to create.                                                                                       |
| `distribution`   | `dict`                                           | `{name: manual}` | Defines how multiple objects are distributed. Support name: `manual`, `random`, `circle`                           |
| `kinematics`     | `dict`                                           | `None`           | Kinematic model of the object. Support name: `diff`, `acker`, `omni`                                               |
| `shape`          | `dict`                                           | `{name: circle}` | Shape of the object.  Support name:  `circle`, `rectangle`, `polygon` , `linestring`                               |
| `state`          | `list` of `float`                                | `[0, 0, 0]`      | Initial state vector of the object.                                                                                |
| `velocity`       | `list` of `float`                                | `[0, 0]`         | Initial velocity vector.                                                                                           |
| `goal`           | `list` of `float` or `list` of `list` of `float` | `None`           | Goal state(s) vector.                                                                                              |
| `behavior`       | `dict`                                           | `None`           | Behavior configuration dictating object movement. Support name: `dash`, `rvo`                                      |
| `group_behavior` | `dict`                                           | `None`           | Group-level behavior for objects in the same group. Support name: `orca`                                           |
| `role`           | `str`                                            | `"obstacle"`     | Role of the object in the simulation.                                                                              |
| `color`          | `str`                                            | `'k'` (black)    | Visualization color of the object in the simulation.                                                               |
| `static`         | `bool`                                           | `False`          | Indicates if the object is static.                                                                                 |
| `vel_min`        | `list` of `float`                                | `[-1, -1]`       | Minimum velocity limits for each control dimension.                                                                |
| `vel_max`        | `list` of `float`                                | `[1, 1]`         | Maximum velocity limits for each control dimension.                                                                |
| `acce`           | `list` of `float`                                | `[inf, inf]`     | Acceleration limits.                                                                                               |
| `angle_range`    | `list` of `float`                                | `[-pi, pi]`      | Range of orientation angles in radians.                                                                            |
| `goal_threshold` | `float`                                          | `0.1`            | Threshold distance to determine goal arrival.                                                                      |
| `sensors`        | `list` of `dict`                                 | `None`           | List of sensor configurations attached to the object. Support name: `lidar2d`                                      |
| `arrive_mode`    | `str`                                            | `'position'`     | Mode for arrival detection.                                                                                        |
| `description`    | `str`                                            | `None`           | Image description or label for the object.                                                                         |
| `group`          | `int`                                            | `0`              | Group identifier for organizational purposes, allowing objects to be grouped.                                      |
| `group_name`     | `str`                                            | `None`           | Group name used for organization, allowing objects to be managed by group name.                                      |
| `unobstructed`   | `bool`                                           | `False`          | Indicates if the object ignores collisions.                                                                        |
| `plot`           | `dict`                                           | `{}`             | Plotting options for object visualization.                                                                         |
| `state_dim`      | `int`                                            | `None`           | Dimension of the state vector.                                                                                     |
| `vel_dim`        | `int`                                            | `None`           | Dimension of the velocity vector.                                                                                  |
| `fov`            | `float`                                          | `None`           | Field of view angles in radians for the object's sensors.                                                          |
| `fov_radius`     | `float`                                          | `None`           | Field of view radius for the object's sensors.                                                                     |

### Detailed Parameter Descriptions

(object-properties)=
::::{dropdown} **object properties**

```{card} Overview
:class-card: sd-bg-light sd-rounded-3
- **`number`** — How many objects to create
- **`distribution`** — Object placement (`manual`, `random`, `circle`)
- **`state`** — Initial position (`[x, y, θ]`)
- **`goal`** — Target destination (`[x, y, θ]`)
- **`velocity`** — Initial speed (`[v, ω]`, `[vx, vy]`, `[v, φ]`)
- **`state_dim`** — State vector size (auto: 3 or 4)
- **`vel_dim`** — Velocity vector size (auto: 2)
- **`name`** — Unique identifier for the object.
- **`role`** — Object type (`robot`, `obstacle`)
```

**`number`** (`int`, default: `1`)
: Specifies the number of objects to create using the given configuration.

  ```yaml
  # Example usage
  robot:
    - number: 5
  ```

**`distribution`** (`dict`, default: `{name: manual}`) ([source](https://ir-sim.readthedocs.io/en/dev/irsim.world.html#irsim.world.object_factory.ObjectFactory.generate_state_list))
: Defines how multiple objects are spatially distributed when `number` is greater than `1`. 

  **Options:**
  - `'manual'`: Manually specify initial states and goals for each object. 
    - In this case, the `state` (or goal) parameters must be provided for each object. If the provided list is shorter than the number of objects, the last state (or goal) is repeated.

    ```yaml
    # Example usage
    distribution: {name: 'manual'}
    state: [[1, 1, 0], [2, 2, 0], [3, 3, 0]]
    goal: [[9, 9, 0], [8, 8, 0], [7, 7, 0]]
    ```

  - `'random'`: Randomly distribute objects within specified ranges. Optional parameters:
    - `range_low` (list): Lower bounds for random distribution. Default is `[0, 0, -3.14]`.
    - `range_high` (list): Upper bounds for random distribution. Default is `[10, 10, 3.14]`. 

    ```yaml
    # Example usage
    distribution: {name: 'random', range_low: [0, 0, -3.14], range_high: [10, 10, 3.14]}
    ```

  - `'circle'`: Arrange objects in a circular formation around a specified center. Optional parameters:
    - `center` (list): Center coordinates of the circle. Default is `[5, 5, 0]`.
    - `radius` (float): Radius of the circle. Default is `4.0`.  

    ```yaml
    # Example usage
    distribution: {name: 'circle', center: [5, 5, 0], radius: 4.0}
    ```

**`state`** (`list` of `float`, default: `[0, 0, 0]`)
: Defines the initial state of the object, typically in the format `[x, y, theta]`, where `theta` represents the orientation in radians. If the provided state has more elements than required, extra elements are truncated; if fewer, missing values are filled with zeros.

  ```yaml
  # Example usage
  state: [1.0, 1.0, 0.2]
  ```

**`velocity`** (`list` of `float`, default: `[0, 0]`)
: Specifies the initial velocity (list) of the object. The format depends on the kinematics model:

  **Format by Kinematics:**
  - For `'diff'`: `[v, omega]`, where `v` is linear velocity and `omega` is angular velocity.
  - For `'omni'`: `[vx, vy]`, velocities along the x and y axes.
  - For `'acker'`: Typically `[v, phi]`, where `v` is linear velocity and `phi` is steering angle.

  ```yaml
  # Example usage
  velocity: [1.0, 0.5]
  ```

**`goal`** (`list` of `float` or `list` of `list` of `float`, default: `None`)
: Sets the target state or position the object should move toward. Used in conjunction with behaviors to guide the object's navigation. The format is `[x, y, theta]` or `[[x, y, theta], [x, y, theta], ...]` for multiple goals.

  ```yaml
  # Example usage - single goal
  goal: [10.0, 10.0, 0.2]
  ```

  **Note**: For multiple goals for the single object (Pay attention to the difference between the single goal for multiple objects and multiple goals for the single object):

  ```yaml
  # Example usage - multiple goals
  goal: 
    - [[10.0, 10.0, 0.2], [5.0, 4.0, 1.0], [3.0, 3.0, 2.0]]
  ```

**`state_dim`** (`int`, default: `None`)
: Explicitly defines the dimension of the state vector. When not specified, this is automatically inferred from the kinematics model. For most use cases, the default inference is sufficient.

  **Common Values:**
  - `3`: For 2D position and orientation `[x, y, theta]`
  - `4`: For vehicles with additional state (e.g., Ackermann with `[x, y, theta, steer_angle]`)

  ```yaml
  # Example usage
  state_dim: 3
  ```

**`vel_dim`** (`int`, default: `None`)
: Explicitly defines the dimension of the velocity vector. When not specified, this is automatically inferred from the kinematics model. The velocity dimension depends on the control inputs for the specific kinematics.

  **Common Values:**
  - `2`: For differential drive `[v, omega]` or omnidirectional `[vx, vy]`
  - Additional dimensions may be used for more complex kinematics

  ```yaml
  # Example usage
  vel_dim: 2
  ```

**`group`** (`int`, default: `0`)
: Specifies a group identifier for organizational purposes, allowing objects to be categorized and managed together. Objects with the same group ID can be treated as a cohesive unit for certain operations.

  ```yaml
  # Example usage
  group: 1
  ```

**`group_name`** (`str`, default: `None`)
: Specifies a group name for organizational purposes, allowing objects to be categorized and managed together. Objects with the same group name can be treated as a cohesive unit for certain operations.

  ```yaml
  # Example usage
  group_name: "robot_flow"
  ```

**`name`** (`str` or `list` of `str`, default: `None`)
: Sets a unique identifier for the object. If not provided, the name defaults to "<role>_<id>" (e.g., `robot_0`, `obstacle_3`). Names must be unique across all objects in the environment; duplicates will raise a `ValueError` during initialization.

  - When creating multiple objects (`number > 1`), you can provide a list of names. If the provided list is shorter than the number of objects, the last name is repeated for remaining objects; ensure uniqueness to avoid errors.

  ```yaml
  # Example usage - single object
  robot:
    - name: "r1"
      kinematics: {name: 'diff'}
      shape: {name: 'circle', radius: 0.2}
      state: [1, 1, 0]
      goal: [9, 9, 0]
  ```

  ```yaml
  # Example usage - multiple objects with explicit names
  robot:
    - number: 3
      name: ["r1", "r2", "r3"]
      distribution: {name: 'circle', center: [5, 5, 0], radius: 4.0}
      kinematics: {name: 'diff'}
      shape: {name: 'circle', radius: 0.2}
  ```

  ```{note}
  Do not confuse the object-level `name` with the `name` keys inside dictionaries like `shape`, `kinematics`, or `behavior`. The latter specify the type of that component, not the object's identifier.
  ```

  **`role`** (`str`, default: `'obstacle'`)
  : Defines the object's role in the simulation, determined by the section it belongs to:
  - `'robot'`: An active entity typically controlled by behaviors or input commands.
  - `'obstacle'`: A passive entity that may or may not move but is considered during collision detection.
::::

(object-kinematics)=
::::{dropdown} **object kinematics**

```{card} Kinematics Models
:class-card: sd-bg-light sd-rounded-3
- **`diff`** — Differential drive, controlled by linear speed and angular velocity (`[v, omega]`)
- **`omni`** — Omnidirectional, controlled by linear speed along the x and y axes (`[vx, vy]`)
- **`acker`** — Ackermann steering, controlled by linear speed and steering angle (`[v, phi]`)
```

**`kinematics`** (`dict`, default: `None`)
: Sets the kinematic model governing the object's movement.

  **Options:**
  - `'diff'`: Differential drive robot, suitable for robots that can rotate in place (e.g., two-wheel robots). This type of robot is controlled by linear and angular velocity. Optional parameters:
    - `noise` (bool): whether to add noise to the velocity commands. Default is `False`.
    - `alpha` (list): noise parameters for velocity commands. Default is `[0.03, 0, 0, 0.03]`.    

    ```yaml
    # Example usage
    kinematics: {name: 'diff', noise: True, alpha: [0.03, 0, 0, 0.03]}
    ```

  - `'omni'`: Omnidirectional movement, allowing movement in any direction without changing orientation. This type of robot is controlled by velocities along the x and y axes. Optional parameters:
    - `noise` (bool): whether to add noise to the velocity commands. Default is `False`.
    - `alpha` (list): noise parameters for velocity commands. Default is `[0.03, 0, 0, 0.03]`.   

    ```yaml
    # Example usage
    kinematics: {name: 'omni', noise: True, alpha: [0.03, 0, 0, 0.03]}
    ```
   
  - `'acker'`: Ackermann steering, typical for car-like vehicles requiring a turning radius.
    - `noise` (bool): whether to add noise to the velocity commands. Default is `False`.
    - `alpha` (list): noise parameters for velocity commands. Default is `[0.03, 0, 0, 0.03]`.  
    - `mode` (str): steering mode, either `steer` or `angular`. Default is `steer`.
      - `steer`: the object is controlled by linear and steer angle.
      - `angular`: the object is controlled by linear and angular velocity. 

    ```yaml
    # Example usage
    kinematics: {name: 'acker', noise: True, alpha: [0.03, 0, 0, 0.03], mode: 'steer'}
    ```
  
**`vel_min`** (`list` of `float`, default: `[-1, -1]`) and **`vel_max`** (`list` of `float`, default: `[1, 1]`)
: Set the minimum and maximum velocity limits for each control dimension (e.g., linear and angular velocities). These constraints ensure the object's motion stays within feasible and safe bounds.

**`acce`** (`list` of `float`, default: `[inf, inf]`)
: Defines acceleration limits as the maximum change in velocity per time step for each control dimension. This parameter simulates the physical limitations of the object's motion capabilities.

**`angle_range`** (`list` of `float`, default: `[-pi, pi]`)
: Specifies the allowed range of orientation angles `[min, max]` in radians. The object's orientation angle `theta` is wrapped within this range to maintain consistency.

**`goal_threshold`** (`float`, default: `0.1`)
: Determines the proximity threshold to the goal at which the object is considered to have arrived. Once within this distance, arrival behaviors or state changes may be triggered.

  ```yaml
  # Example usage
  vel_min: [-1, -1]
  vel_max: [1, 1]
  acce: [0.5, 0.1]
  angle_range: [-pi, pi]
  goal_threshold: 0.1
  ```

  ````{warning}
    When using the `acker` kinematics model, ensure that the `wheelbase` parameter is set in the `shape` configuration.
  ````
::::

(object-shape)=
::::{dropdown} **object shape**

```{card} Overview
:class-card: sd-bg-light sd-rounded-3
- **`circle`** — Round shape (`radius`, `center`)
- **`rectangle`** — Rectangular shape (`length`, `width`, `wheelbase`)
- **`polygon`** — Custom shape (`vertices`, `is_convex`)
- **`linestring`** — Line segments (`vertices`)
```

**`shape`** (`dict`, default: `{name: circle}`)
: Determines the geometric shape used for collision detection and visualization in the original state. 

  **Supported Shapes:**

  - **`'circle'`**: Represents a circular shape.
    - **`radius`** (`float`): Radius of the circle. Default is `0.2`.
    - **`center`** (`list`): Center (x, y) of the circle. Default is `[0, 0]`.
    - **`random_shape`** (`bool`): Whether to generate a random radius. Default is `False`.
    - **`radius_range`** (`list`): Range `[min_radius, max_radius]` for random radius generation if `random_shape` is `True`. Default is `[0.1, 1.0]`.
    - **`wheelbase`** (`float`): Wheelbase of the Ackermann steering vehicle. Required when using `'acker'` kinematics. Default is `None`.

    ```yaml
    # Example usage
    shape: {name: 'circle', radius: 0.2, center: [0, 0]}
    ```

  - **`'rectangle'`**: Represents a rectangular shape.
    - **`length`** (`float`): Length of the rectangle along the x-axis. Default is `1.0`.
    - **`width`** (`float`): Width of the rectangle along the y-axis. Default is `1.0`.
    - **`wheelbase`** (`float`): Wheelbase of the Ackermann steering vehicle. Required when using `'acker'` kinematics. Default is `None`.

    ```yaml
    # Example usage
    shape: {name: 'rectangle', length: 1.0, width: 0.5}
    ```
  
  - **`'polygon'`**: Represents a polygonal shape defined by a list of vertices.
    - **`vertices`** (`list`): List of vertices defining the polygon in the format `[[x1, y1], [x2, y2], ...]`, if not provided, a random polygon will be generated.
    - **`random_shape`** (`bool`): Whether to generate a series of random polygons. Default is `False`.
    - **`is_convex`** (`bool`): Whether to generate a series of random convex polygons. Default is `False`.
    - parameters for random polygon generation, see [random_generate_polygon](#irsim.lib.algorithm.generation.random_generate_polygon) for more details. Parameters include `number `, `center_range `, `avg_radius_range `, `irregularity_range `, `spikeyness_range `, `num_vertices_range `.
      
    ```yaml
    # Example usage
    shape:
      name: 'polygon'
      vertices: 
        - [4.5, 4.5]
        - [5.5, 4.5]
        - [5.5, 5.5]
        - [4.5, 5.5]
    ```

    ```yaml
    # Example usage - random polygon
    shape:
      - {name: 'polygon', random_shape: true, center_range: [5, 10, 40, 30], avg_radius_range: [0.5, 2], irregularity_range: [0, 1], spikeyness_range: [0, 1], num_vertices_range: [4, 5]} 
    ```
  
  - **`'linestring'`**: Represents a line string shape defined by a list of vertices. Similar to a polygon but generates a line string.
    - **`vertices`** (`list`): List of vertices defining the line string in the format `[[x1, y1], [x2, y2], ...]`.
    - **`random_shape`** (`bool`): Whether to generate a series of random line strings (polygon). Default is `False`.
    - **`is_convex`** (`bool`): Whether to generate a series of random convex line strings (polygons). Default is `False`.
    - parameters for random line string generation (polygon), see [random_generate_polygon](#irsim.lib.algorithm.generation.random_generate_polygon) for more details. Parameters include `number `, `center_range `, `avg_radius_range `, `irregularity_range `, `spikeyness_range `, `num_vertices_range `.

    ```yaml
    # Example usage
    shape:
      name: 'linestring'
      vertices: 
        - [4.5, 4.5]
        - [5.5, 4.5]
        - [5.5, 5.5]
        - [4.5, 5.5]
    ``` 

    ```yaml
    # Example usage - random linestring
    shape:
      - {name: 'linestring', random_shape: true, center_range: [5, 10, 40, 30], avg_radius_range: [0.5, 2], irregularity_range: [0, 1], spikeyness_range: [0, 1], num_vertices_range: [4, 5]} 
    ```
::::

(object-behavior)=
::::{dropdown} **object behavior**

```{card} Behavior Systems
:class-card: sd-bg-light sd-rounded-3
- **`behavior`**: `dash` (move toward the goal directly), `rvo` (collision avoidance algorithm)
- **`group_behavior`**: `orca` (optimal reciprocal collision avoidance)
- **`static`** — Immobile objects (`True`/`False`)
```

**`behavior`** (`dict`, default: `None`)
: Configures the movement behavior of the object. Behaviors can be simple or complex and may include additional parameters.

  **Options:**
  - `'dash'`: Moves directly toward the goal at maximum allowable speed.
    - `wander` (bool/`False`): Whether to add random wandering to the movement. If `True`, the object will have a random goal when reach current goal.
    - `target_roles` (str/`all`): Only the objects with the target role will be applied to the behavior. Currently, you can set the target role as `robot` or `obstacle`.
    - `range_low`(list/`[0, 0, -3.14]`): Lower bounds for random wandering. 
    - `range_high`(list/`[10, 10, 3.14]`): Upper bounds for random wandering. 
    - `angle_tolerance` (float/`0.1`): Tolerance for orientation alignment with `diff` and `acker` kinematics. 
  
    **Example:**
    ```yaml
    behavior: {name: 'dash', wander: True, range_low: [0, 0, -3.14], range_high: [10, 10, 3.14], angle_tolerance: 0.1}
    ```

  - `'rvo'`: Implements Reciprocal Velocity Obstacles for collision avoidance among multiple moving objects. Support kinematics are `diff` and `omni`.
    - `wander` (bool/`False`): Whether to add random wandering to the movement. If `True`, the object will have a random goal when reach current goal.
    - `target_roles` (str/`all`): Only the objects with the target role will be applied to the behavior. Currently, you can set the target role as `robot` or `obstacle`.
    - `range_low`(list/`[0, 0, -3.14]`): Lower bounds for random wandering.
    - `range_high`(list/`[10, 10, 3.14]`): Upper bounds for random wandering.
    - `vxmax` (float/`1.5`): Maximum linear velocity in x axis.
    - `vymax` (float/`1.5`): Maximum linear velocity in y axis.
    - `acce` (float/`1.0`): Maximum acceleration.
    - `factor` (float/`1.0`): Factor for the RVO algorithm.
    - `mode` (str/`rvo`): Mode for RVO algorithm, either `rvo`, `hrvo`, or `vo`.
      - `rvo`: Reciprocal Velocity Obstacles. For multi-agent collision avoidance.
      - `hrvo`: Hybrid Reciprocal Velocity Obstacles. Combine RVO with VO to avoid deadlocks.
      - `vo`: Velocity Obstacles. For obstacle avoidance.
    - `neighbor_threshold` (float/`3.0`): Distance threshold to filter the neighbors to the self robot.

    **Example:**
    ```yaml
    behavior: {name: 'rvo', vxmax: 1.5, vymax: 1.5, acce: 1.0, factor: 1.0, mode: 'rvo', wander: False}
    ```

**`group_behavior`** (`dict`, default: `None`)
: Group-level behavior for all objects within the same group. This is more efficient for coordinated behaviors (like swarm or crowd simulation) as it computes actions for all members in a single step. Must include `name` plus behavior-specific parameters.

  **Options:**
  - `'orca'`: Optimal Reciprocal Collision Avoidance (ORCA) for multi-agent navigation. Requires `pyrvo` package (`pip install pyrvo`). The simulation will attempt to import it only when this behavior is used.
    - `wander` (bool/`False`): Whether to add random wandering to the movement. If `True`, the object will have a random goal when reach current goal.
    - `range_low`(list/`[0, 0, -3.14]`): Lower bounds for random wandering.
    - `range_high`(list/`[10, 10, 3.14]`): Upper bounds for random wandering.
    - `neighborDist` (float/`15.0`): Max distance to search for neighbors.
    - `maxNeighbors` (int/`10`): Max number of neighbors to consider.
    - `timeHorizon` (float/`20.0`): Time horizon for computing safe velocities with respect to other agents.
    - `timeHorizonObst` (float/`10.0`): Time horizon for computing safe velocities with respect to static obstacles.
    - `safe_radius` (float/`0.1`): Additional safety radius padding.
    - `maxSpeed` (float/`None`): Max speed for the agents. If `None`, uses the object's `vel_max`.

    **Example:**
    ```yaml
    group_behavior: 
      name: 'orca'
      neighborDist: 10.0
      maxNeighbors: 10
      timeHorizon: 10.0
      timeHorizonObst: 10.0
      safe_radius: 0.1
    ```

**`static`**:
  A boolean indicating whether the object is static (does not move). Static objects ignore kinematics and behaviors, remaining at their initial state.

  **Example:**
  ```yaml
  static: True
  ```
::::

(object-sensor)=
::::{dropdown} **object sensors**

```{card} Overview
:class-card: sd-bg-light sd-rounded-3
- **`lidar2d`** — 2D laser scanner (`range_min/max`, `angle_range`, `noise`)
- **`fov`** — Field of view angle (radians)
- **`fov_radius`** — Maximum detection distance
```

**`sensors`**:
  Attaches sensors to the object for environmental perception. Each sensor is defined by a dictionary indicating its type and specific parameters. Currently supported sensor `name` (or `type`) include:
  - `lidar2d`: 2D LiDAR sensor for distance measurements. Parameters include:
    - `range_min` (float/`0.0`): Minimum detection range.
    - `range_max` (float/`10.0`): Maximum detection range.
    - `angle_range` (float/`pi`): Total angle range of the sensor.
    - `number` (int/`100`): Number of laser beams.
    - `scan_time` (float/`0.1`): Time taken for one complete scan.
    - `noise` (bool/`False`): Whether noise is added to measurements.
    - `std` (float/`0.2`): Standard deviation for range noise if `noise` is `True`.
    - `angle_std` (float/`0.02`): Standard deviation for angle noise if `noise` is `True`.
    - `offset` (list/`[0, 0, 0]`): Offset of the sensor from the object's position (x, y, theta).
    - `alpha` (float/`0.3`): Transparency for plotting.
    - `has_velocity` (bool/`False`): Whether measures the lidar point velocity.
    - `color` (str/`r`): Color of the sensor.

    **Example:**
    ```yaml
    
    sensors:
      - name: 'lidar2d'
        range_min: 0
        range_max: 5
        angle_range: 3.14 
        number: 200
        noise: False
        std: 0.2
        angle_std: 0.2
        offset: [0, 0, 0]
        alpha: 0.3
    ```

**`fov`** and **`fov_radius`**:
  Define the field of view (FOV) for the object's sensors. The FOV is the angular range within which the sensor can detect objects. The `fov` parameter specifies the angular range in radians, while `fov_radius` sets the maximum detection distance.

  **Example:**
  ```yaml
  fov: 1.57
  fov_radius: 5.0
  ```
::::

(object-mode)=
::::{dropdown} **object mode**

```{card} Overview
:class-card: sd-bg-light sd-rounded-3
- **`arrive_mode`** — Goal detection (`position`, `state`)
- **`unobstructed`** — Ignore collisions (`True`/`False`)
```

**`arrive_mode`** (`str`, default: `'position'`)
: Chooses the method for determining if the object has arrived at its goal:

  **Options:**
  - `'position'`: Arrival is based solely on proximity to the goal position (`[x, y]`).
  - `'state'`: Considers both position and orientation in the arrival check (`[x, y, theta]`).

  **Example:**
  ```yaml
  arrive_mode: 'position'
  ```

**`unobstructed`** (`bool`, default: `False`)
: When set to `True`, this object is treated as having an unobstructed path, ignoring collisions with other objects and obstacles. This can be useful for testing or for objects that must not be impeded.

  **Example:**
  ```yaml
  unobstructed: True
  ```
::::

(object-visualization)=
::::{dropdown} **object visualization**

```{card} Overview
:class-card: sd-bg-light sd-rounded-3
- **`color`** — Object color (`'r'`, `'blue'`, `'k'`)
- **`description`** — Image file (`'car_blue.png'`, `'diff_robot0.png'`)
- **`plot`** — Advanced display options
  - **Object** — Appearance (`obj_color`, `obj_alpha`, `obj_linestyle`)
  - **Goal** — Goal markers (`show_goal`, `goal_color`, `show_goal_text`)
  - **Trail** — Object trails (`show_trail`, `keep_trail_length`)
  - **Trajectory** — Path lines (`show_trajectory`, `keep_traj_length`)
  - **Sensors** — Sensor display (`show_sensor`, `show_fov`)
```

**`color`** (`str`, default: `'k'` (black))
: Specifies the object's color in visualizations for easy identification. Detailed color options can be found in [matplotlib color](https://matplotlib.org/stable/gallery/color/named_colors.html).

  **Example:**
  ```yaml
  color: 'r'
  ```

**`description`** (`str`, default: `None`)
: Provides an image for representing the object graphically. Supports image files located in world/description. You can also set the absolute path of the image file by your need.

  **Available Images:**
  - `car_green.png`: A default image for the ackermann steering vehicle.
  - `car_blue.png`
  - `car_red.png`
  - `diff_robot0.png`
  - `diff_robot1.png`
  
  **Example:**
  ```yaml
  description: 'car_blue.png'
  ```

**`plot`**:
  Contains plotting options controlling the visual representation of the object. All plot elements are initially created at the origin and positioned using transforms and data updates during animation updates.

  **Object Visualization Properties:**
  - `obj_linestyle` (str/`'-'`): Line style for object outline (e.g., '-', '--', ':', '-.').
  - `obj_zorder` (int/`3`): Z-order (drawing layer) for object elements. Default is 3 for robots, 1 for obstacles.
  - `obj_color` (str): Color of the object. Default is the object's color property.
  - `obj_alpha` (float/`1.0`): Transparency of the object (0.0 to 1.0).
  - `obj_linewidth` (float/`None`): Width of the object outline. Default varies by object type.

  **Goal Visualization:**
  - `show_goal` (bool/`False`): Whether to show the goal position.
    - `goal_color` (str): Color of the goal marker. Default is the object's color.
    - `goal_alpha` (float/`0.5`): Transparency of the goal marker (0.0 to 1.0).
    - `goal_zorder` (int/`1`): Z-order of the goal marker.
  - `show_goal_text` (bool/`False`): Whether to show the goal text.

  **Text Label Visualization:**
  - `show_text` (bool/`False`): Whether to show text information.
    - `text_color` (str): Color of the text. Default is 'k' (black).
    - `text_size` (int/`10`): Font size of the text.
    - `text_alpha` (float/`1.0`): Transparency of the text (0.0 to 1.0).
    - `text_zorder` (int/`2`): Z-order of the text.
    - `text_position` (list/`[-radius-0.1, radius+0.1]`): Position offset from object center [dx, dy].

  **Velocity Arrow Visualization:**
  - `show_arrow` (bool/`False`): Whether to show the velocity arrow.
    - `arrow_color` (str/`"gold"`): Color of the arrow.
    - `arrow_length` (float/`0.4`): Length of the arrow.
    - `arrow_width` (float/`0.6`): Width of the arrow.
    - `arrow_alpha` (float/`1.0`): Transparency of the arrow (0.0 to 1.0).
    - `arrow_zorder` (int/`4`): Z-order of the arrow.

  **Trajectory Path Visualization:**
  - `show_trajectory` (bool/`False`): Whether to show the trajectory line.
    - `traj_color` (str): Color of the trajectory. Default is the object's color.
    - `traj_style` (str): Line style of the trajectory (e.g., '-', '--', ':', '-.'). Default is "-".
    - `traj_width` (float/`None`): Width of the trajectory line. Default is the object's width.
    - `traj_alpha` (float/`0.5`): Transparency of the trajectory (0.0 to 1.0).
    - `traj_zorder` (int/`0`): Z-order for trajectory elements.
    - `keep_traj_length` (int/`0`): Number of steps to keep from the end of trajectory. Default is 0 (keep all steps).

  **Object Trail Visualization:**
  - `show_trail` (bool/`False`): Whether to show object trails.
    - `trail_freq` (int/`2`): Frequency of trail display (every N steps).
    - `trail_type` (str): Type of trail shape. Default is the object's shape.
    - `trail_edgecolor` (str): Edge color of the trail. Default is the object's color.
    - `trail_linewidth` (float/`0.8`): Width of the trail outline.
    - `trail_alpha` (float/`0.7`): Transparency of the trail (0.0 to 1.0).
    - `trail_fill` (bool/`False`): Whether to fill the trail shape.
    - `trail_color` (str): Fill color of the trail. Default is the object's color.
    - `trail_zorder` (int/`0`): Z-order for trail elements.
    - `keep_trail_length` (int/`0`): Number of steps to keep from the end of trail. Default is 0 (keep all steps).

  **Sensor Visualization:**
  - `show_sensor` (bool/`True`): Whether to show sensor visualizations.

  **Field of View Visualization:**
  - `show_fov` (bool/`False`): Whether to show field of view visualization.
    - `fov_color` (str/`"lightblue"`): Fill color of the field of view.
    - `fov_edge_color` (str/`"blue"`): Edge color of the field of view.
    - `fov_alpha` (float/`0.5`): Transparency of the field of view (0.0 to 1.0).
    - `fov_zorder` (int/`1`): Z-order of the field of view.

  **Note:** All visual elements are created at the origin during initialization and positioned using matplotlib transforms (for patches) and set_data methods (for lines) during animation updates.

  **Example:**
  ```yaml
  plot:
    # Object appearance
    obj_linestyle: '--'
    obj_zorder: 3
    obj_color: 'blue'
    obj_alpha: 0.8
    obj_linewidth: 2.0
    
    # Goal visualization
    show_goal: True
    goal_color: 'red'
    goal_alpha: 0.7
    goal_zorder: 2
    
    # Text labels
    show_text: True
    text_color: 'black'
    text_size: 12
    text_alpha: 0.9
    text_zorder: 5
    
    # Velocity arrows
    show_arrow: True
    arrow_color: 'gold'
    arrow_length: 0.5
    arrow_width: 0.8
    arrow_alpha: 0.9
    arrow_zorder: 4
    
    # Trajectory path
    show_trajectory: True
    traj_color: 'green'
    traj_style: '-'
    traj_width: 0.6
    traj_alpha: 0.6
    traj_zorder: 1
    
    # Object trails
    show_trail: True
    trail_freq: 3
    trail_edgecolor: 'purple'
    trail_linewidth: 1.0
    trail_alpha: 0.5
    trail_fill: False
    trail_color: 'purple'
    trail_zorder: 0
    
    # Sensors and FOV
    show_sensor: True
    show_fov: True
    fov_color: 'lightblue'
    fov_edge_color: 'blue'
    fov_alpha: 0.3
    fov_zorder: 1
  ```
::::

---

## GUI Configuration

(gui-parameters-table)=
### GUI Parameters Table

| Parameter  | Type   | Default | Description                                                               |
| ---------- | ------ | ------- | ------------------------------------------------------------------------- |
| `keyboard` | `dict` | `{}`    | Keyboard control options (effective when `world.control_mode: keyboard`). |
| `mouse`    | `dict` | `{}`    | Mouse control options.                                                    |

(keyboard-properties)=
:::::{dropdown} **keyboard properties**

**`keyboard`** (`dict`, default: `{}`)
Configure keyboard control. Options are read by `KeyboardControl`. The default backend is `pynput`. If `pynput` is unavailable, IR‑SIM automatically falls back to the Matplotlib backend.

  - `backend` (`str`/`"pynput"`): Keyboard backend.
    - `"pynput"`: Global keyboard hook, but works when the Matplotlib window is focused (requires the `pynput` package).
    - `"mpl"`: Matplotlib figure key events works when the figure window is focused. (no extra dependency, but may be delayed when there are lots of objects).
  - `global_hook` (`bool`/`False`): With `backend: 'pynput'`, capture keys even if the Matplotlib window is not focused. (only active when the window is focused).
  - `key_lv_max` (`float`/`3.0`): Maximum linear velocity.
  - `key_ang_max` (`float`/`1.0`): Maximum angular velocity.
  - `key_lv` (`float`/`0.0`): Initial linear velocity.
  - `key_ang` (`float`/`0.0`): Initial angular velocity.
  - `key_id` (`int`/`0`): Initial robot control id.

```yaml
# Example: enable keyboard control with GUI settings
world:
  control_mode: 'keyboard'

gui:
  keyboard:
    backend: 'pynput'     # or 'mpl'
    global_hook: true     # if your want to capture keys globally. Default is False.
    key_id: 0
    key_lv_max: 3.0
    key_ang_max: 1.0
```

**Keyboard Control Keys:**

- `w` — Forward
- `s` — Backward
- `a` — Turn Left
- `d` — Turn Right
- `q` — Decrease Linear Velocity
- `e` — Increase Linear Velocity
- `z` — Decrease Angular Velocity
- `c` — Increase Angular Velocity
- `alt+num` — Change Current Control Robot ID
- `r` — Reset the Environment
- `space` — Toggle Pause/Resume Environment
- `esc` — Quit the Environment (sets quit flag)
- `x` — Switch Keyboard/Auto Control
- `l` — Reload the Environment
- `F5` — Debug the Environment (fn+f5 for mac)
- `v` — Save the current figure

```{tip}
Debug mode lets you step the simulation frame-by-frame for inspection:

- Press `F5` to enter debug mode at the current frame. While in debug mode, pressing `F5` again advances by one frame (single-step).
- Press `space` to exit debug mode and resume normal execution.

Notes:
- On macOS laptops, you may need to hold the `fn` key (e.g., `fn+F5`). `alt` key is corresponding to `option` key in macOS. 
```

:::::

(mouse-properties)=
:::::{dropdown} **mouse properties**

**`mouse`** (`dict`, default: `{}`)
:: Configure mouse control. Options are read by `MouseControl`.

  - `zoom_factor` (`float`): Zoom factor. Default is `1.1`.


  ```yaml
  gui:
    mouse:
      zoom_factor: 1.1
  ``` 

  ````{note}
  **Mouse Actions:**

  - **Mouse Movement** — Track mouse position and update display coordinates
  - **Middle Click** — Reset zoom to default view
  - **Scroll Up** — Zoom in (centered on mouse position)
  - **Scroll Down** — Zoom out (centered on mouse position)

  **Mouse Position Attributes:**
  - `mouse_left_pos` (`tuple`): Position of left click (x, y).
  - `mouse_right_pos` (`tuple`): Position of right click (x, y).
  - `mouse_pos` (`tuple`): Current mouse position (x, y).
  ````
:::::

---

## Configuration Examples

Let's explore various configuration examples to demonstrate the flexibility and power of IR-SIM:

::::{tab-set}

:::{tab-item} Multi-Robot RVO
```yaml
robot:
  - number: 10
    distribution: {name: 'circle', radius: 4.0, center: [5, 5]}  
    kinematics: {name: 'diff'}
    shape: 
      - {name: 'circle', radius: 0.2}  
    behavior: {name: 'rvo', vxmax: 1.5, vymax: 1.5, acce: 1.0, factor: 1.0}
    vel_min: [-3, -3.0]
    vel_max: [3, 3.0]
    color: ['royalblue', 'red', 'green', 'orange', 'purple', 'yellow', 'cyan', 'magenta', 'lime', 'pink', 'brown'] 
    arrive_mode: position
    goal_threshold: 0.15
    plot:
      show_trail: true
      show_goal: true
      trail_fill: true
      trail_alpha: 0.2
      show_trajectory: false
```
:::

:::{tab-item} Multi-Robot ORCA
```yaml
robot:
  - number: 10
    distribution: {name: 'circle', radius: 4.0, center: [5, 5]}
    kinematics: {name: 'omni'}
    shape: {name: 'circle', radius: 0.2}
    group_behavior: 
      name: 'orca'
      neighborDist: 10.0
      maxNeighbors: 10
      timeHorizon: 10.0
      timeHorizonObst: 10.0
      safe_radius: 0.1
    vel_max: [2.0, 2.0]
    goal: [9, 9, 0]
    plot:
      show_trail: true
      trail_alpha: 0.5
```
:::

:::{tab-item} Mixed Obstacles
```yaml
obstacle:
  - shape: {name: 'circle', radius: 1.0}  # radius
    state: [5, 5, 0]  
  
  - shape: {name: 'rectangle', length: 1.5, width: 1.2}  # radius
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

:::{tab-item} Ackermann Vehicle
```yaml
robot:  
  - kinematics: {name: 'acker'}  
    shape: {name: 'rectangle', length: 4.6, width: 1.6, wheelbase: 3}
    state: [1, 1, 0, 0]
    goal: [40, 40, 0]
    vel_max: [4, 1]
    behavior: {name: 'dash'}
    plot:
      show_trajectory: True
```
:::

:::{tab-item} Sensor Integration
```yaml
robot:
  - kinematics: {name: 'diff'}
    shape: {name: 'circle', radius: 0.3}
    state: [2, 2, 0]
    goal: [8, 8, 0]
    behavior: {name: 'rvo'}
    sensors:
      - name: 'lidar2d'
        range_min: 0.1
        range_max: 8.0
        angle_range: 6.28  # Full 360 degrees
        number: 360
        noise: True
        std: 0.1
        offset: [0, 0, 0]
        color: 'red'
    plot:
      show_sensor: True
      show_fov: True
      fov_color: 'lightgreen'
      fov_alpha: 0.3
```
:::
::::

````{tip}
**Configuration Best Practices:**

- **Multiple Objects**: When configuring multiple objects, use the `number` and `distribution` parameters to efficiently generate them. For instance, setting `number: 10` with a `distribution` of `'random'` can quickly populate the simulation with randomly placed objects. 
- **Dictionary Parameters**: All dictionary-type parameters (e.g., `distribution`, `shape`, `kinematics`, `behavior`) must include a `'name'` key to specify their type. Omitting the `'name'` key will result in default values or errors.
- **Group Configurations**: By default, objects within the same group share configurations. To customize individual objects within a group, add sub-parameters using `-`. Unspecified objects will inherit the last defined configuration within the group.
- **Kinematics and Velocities**: Ensure that the `velocity` and `vel_max` parameters match the kinematics model. For example, a differential drive robot (`'diff'`) should have velocities in `[v, omega]`, while an omnidirectional robot (`'omni'`) uses `[vx, vy]`.
- **Plotting Options**: Customize the visualization of your simulation through the `plot` parameter for each object if the `plot` section is located in the object configuration. If it is located in the root of the object configuration, it will be applied to all objects.
````








