# YAML Configuration Syntax

The configuration file is a YAML file to initialize the environment. It contains the configuration parameters to simulate and visualize the world, obstacle, and robot. You can customize the scenario and define the behavior of the objects simply by using the parameters.

---

## Parameter Quick Reference

Use this navigation to quickly jump to specific parameter sections:

::::{dropdown} **World Parameters**
:color: primary
:icon: globe
:open:

- [world properties](#world-properties)
- [world map](#world-map)
- [world visualization](#world-visualization)
- [world mode](#world-mode)
::::

::::{dropdown} **Object Parameters**
:color: info
:icon: server
:open:

- [object properties](#object-properties)
  - `number`, `distribution`, `state`, `goal`, `velocity`
  - `role`, `color`, `static`, `arrive_mode`
  - `vel_min`, `vel_max`, `acce`, `angle_range`
- [object kinematics](#object-kinematics)
- [object shape](#object-shape)
- [object behavior](#object-behavior)
- [object sensors](#object-sensor)
- [object visualization](#object-visualization)
::::

:::
::::

---

## Quick Start Example

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
  collision_mode: 'stop'  # 'stop', 'unobstructed', 'reactive', 'unobstructed_obstacles'

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

:::{tab-item} Advanced Configuration
```yaml
world:
  name: "advanced_world"
  height: 20
  width: 20
  step_time: 0.05
  sample_time: 0.1
  control_mode: 'auto'
  collision_mode: 'reactive'
  obstacle_map: 'hm3d_2.png'
  plot:
    show_title: true
    figure_pixels: [1200, 800]

robot:
  - number: 5
    distribution: {name: 'circle', radius: 3.0}
    kinematics: {name: 'diff', noise: true}
    shape: {name: 'circle', radius: 0.3}
    behavior: {name: 'rvo', vxmax: 2.0, vymax: 2.0}
    sensors:
      - name: 'lidar2d'
        range_max: 5.0
        number: 180
    plot:
      show_trail: true
      show_goal: true
      show_sensor: true
```
:::
::::

````{important}
**Key Configuration Guidelines:**

- To include several robots or obstacles in the configuration file, add separate entries under the robot and obstacle sections using `-` for each additional item.
- Parameters such as distribution, shape, behavior, and kinematics must be formatted as `{key: value}` pairs. Ensure that each dictionary includes the `name` key; omitting name will result in a None value for that parameter.
- When dealing with multiple objects (i.e., when the number is greater than 1), utilize the `distribution` parameter to define how these objects are distributed.
- By default, all objects within the same group share identical configurations. To customize individual objects within a group, add sub-parameters using `-`. Any additional objects not explicitly configured will inherit the settings of the last specified object in the group.
````

---

(world-parameters-table)=
### World Parameters Table

| **Parameter**    | **Type**          | **Default** | **Description**                                                                                         |
| ---------------- | ----------------- | ----------- | ------------------------------------------------------------------------------------------------------- |
| `name`           | `str`             | `"world"`   | Name of the world                                                                                       |
| `height`         | `float`           | `10`        | Height of the world (meter)                                                                             |
| `width`          | `float`           | `10`        | Width of the world (meter)                                                                              |
| `step_time`      | `float`           | `0.1`       | Time interval between simulation steps (in seconds)                                                     |
| `sample_time`    | `float`           | `0.1`       | Time interval between samples for rendering and data extraction (in seconds)                            |
| `offset`         | `list` of `float` | `[0, 0]`    | Offset for the world's position in `[x, y]` coordinates                                                 |
| `control_mode`   | `str`             | `"auto"`    | Control mode of the simulation. Support mode: `auto` or `keyboard`                                      |
| `collision_mode` | `str`             | `"stop"`    | Collision handling mode (Support: `"stop"`, `"reactive"`, `"unobstructed"`, `"unobstructed_obstacles"`) |
| `status`         | `str`             | `None` | Initial status of the simulation environment (Support: `"Running"`, `"Arrived"`, `"Collision"`, `"Pause"`)                          |
| `obstacle_map`   | `str` (file path) | `None`      | Path to the image file representing the obstacle map                                                    |
| `mdownsample`    | `int`             | `1`         | Downsampling factor for the obstacle map to reduce resolution and decrease computational load.          |
| `plot`           | `dict`            | `{}`        | Plotting options for initializing the plot of the world.                                                |

### Detailed Parameter Descriptions

(world-properties)=
::::{dropdown} **world properties**

**`name`**:  
  Defines the name of the world used in the simulation. This can be useful for identifying different simulation environments.

**`height`**:  
  Specifies the vertical size of the world in units of meters in the Y-axis direction plotted on the screen.

**`width`**:  
  Specifies the horizontal size of the world in units of meters in the X-axis direction plotted on the screen.

**`step_time`**:  
  Determines the time interval between each simulation step. A smaller `step_time` results in a higher simulation frequency (e.g., `0.1` seconds corresponds to 10 Hz) but needs longer time to run the simulation.

**`sample_time`**:  
  Defines the time interval for rendering the simulation and extracting data. This controls how frequently visual updates and data recordings occur.

**`offset`**:  
  Sets the initial positional offset of the world on the X and Y axes. This is useful for positioning the world within a larger coordinate system or for relative placement.
::::

(world-mode)=
::::{dropdown} **world mode**

**`control_mode`**:  
Configures how the objects in the simulation are controlled:
  - `auto`: Automatic control by the input velocities defined in python script or behavior in the YAML file.
  - `keyboard`: Manual control via keyboard inputs. The key inputs are defined in the file.

**`collision_mode`**:  
  Defines how collisions between objects are handled in the simulation:
  - `stop`: Stops the movement of objects upon collision.
  - `reactive`: Objects react to collisions based on predefined behaviors. 
  - `unobstructed`: Allows objects to pass through each other without consideration of any collision.
  - `unobstructed_obstacles`: Only allows obstacles to pass through each other without consideration of any collision. The robots will stop when they are in collision with the obstacles.

**`status`**:  
  Sets the initial status of the simulation environment:
  - `"Running"`: The simulation runs normally (default).
  - `"Pause"`: The simulation starts in a paused state.
  
  The status can be dynamically changed during simulation using keyboard controls (space key) or programmatically.
::::

(world-map)=
::::{dropdown} **world map**

**`obstacle_map`**:  
  Specifies the file path to an image that serves as the obstacle map. This image is used to generate the grid map that defines the positions of obstacles within the world. Each pixel in the image corresponds to a grid cell in the map, where the color of the pixel determines the presence of an obstacle. 
  
  We provide some example maps in the `irsim/world/map` folder and you can also use your own map by 3D datasets like [HM3D](https://aihabitat.org/datasets/hm3d/), [MatterPort3D](https://niessner.github.io/Matterport/), [Gibson](http://gibsonenv.stanford.edu/database/), etc. See [here](https://github.com/hanruihua/ir-sim/tree/features/irsim/world/map/binary_map_generator_hm3d) for more details.

  **e.g.**
  ```yaml
  obstacle_map: 'hm3d_2.png' # hm3d_1.png, hm3d_2.png, hm3d_3.png, hm3d_4.png, hm3d_5.png, hm3d_6.png, hm3d_7.png, hm3d_8.png, hm3d_9.png, cave.png
  ```

**`mdownsample`**:  
  Sets the downsampling factor for the obstacle map image. A higher value reduces the resolution of the obstacle map, which can optimize the simulation performance by decreasing computational load. 
::::

(world-visualization)=
::::{dropdown} **world visualization**

**`plot`**:  
  Specifies the plotting options for initializing the plot of the world.

  - `saved_figure`: Default dpi is 100; default format is `png`; default bbox_inches is `tight`. See [matplotlib.pyplot.savefig](https://matplotlib.org/stable/api/_as_gen/matplotlib.pyplot.savefig.html) for more details.
  - `figure_pixels`: Width and height of the figure in pixels. Default is [1180, 1080].
  - `show_title`: Whether to show the plot title. Default is True.
  - `title`: Custom title for the plot. If not specified, shows simulation time and status.
  - `no_axis`: Whether to show the axis. Default is False.
  - `tight`: Whether to use tight layout. Default is True.
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
  collision_mode: 'stop'              # Collision handling mode ('stop', 'unobstructed', 'reactive', 'unobstructed_obstacles')
  obstacle_map: "path/to/map.png"     # Path to the obstacle map image file
  mdownsample: 2                      # Downsampling factor for the obstacle map
  status: "Running"                   # Initial simulation status
  plot:                               # Plotting configuration
    show_title: true                  # Show plot title
    title: "Custom Simulation Title"  # Custom title (optional)
    figure_pixels: [1200, 800]       # Figure size in pixels
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

::::{grid} 1 2 2 2
:gutter: 2

:::{grid-item-card} **Basic Properties**
:shadow: md

- **Number & Distribution**: Object count and placement
- **Shape & Kinematics**: Physical properties and movement models
- **State & Goals**: Initial positions and targets
:::

:::{grid-item-card} **Behavior & Control**
:shadow: md

- **Behavior Systems**: Movement algorithms (dash, RVO)
- **Velocity Limits**: Speed and acceleration constraints
- **Collision Handling**: Interaction with other objects
:::

:::{grid-item-card} **Sensors & Perception**
:shadow: md

- **Sensor Configurations**: LiDAR, cameras, and other sensors
- **Field of View**: Detection ranges and angles
- **Noise Models**: Realistic sensor imperfections
:::

:::{grid-item-card} **Visualization**
:shadow: md

- **Plotting Options**: Colors, trails, and visual elements
- **Real-time Display**: Trajectories, goals, and sensor data
- **Custom Styling**: Personalized appearance settings
:::
::::

(object-parameters-overview)=
### Object Parameters Overview

| Parameter        | Type                                             | Default          | Description                                                                              |
| ---------------- | ------------------------------------------------ | ---------------- | ---------------------------------------------------------------------------------------- |
| `number`         | `int`                                            | `1`              | Number of objects to create.                                                             |
| `distribution`   | `dict`                                           | `{name: manual}` | Defines how multiple objects are distributed. Support name: `manual`, `random`, `circle` |
| `kinematics`     | `dict`                                           | `None`           | Kinematic model of the object. Support name: `diff`, `acker`, `omni`                     |
| `shape`          | `dict`                                           | `{name: circle}` | Shape of the object.  Support name:  `circle`, `rectangle`, `polygon` , `linestring`     |
| `state`          | `list` of `float`                                | `[0, 0, 0]`      | Initial state vector of the object.                                                      |
| `velocity`       | `list` of `float`                                | `[0, 0]`         | Initial velocity vector.                                                                 |
| `goal`           | `list` of `float` or `list` of `list` of `float` | `[10, 10, 0]`    | Goal state(s) vector.                                                                    |
| `behavior`       | `dict`                                           | `None`           | Behavior configuration dictating object movement. Support name: `dash`, `rvo`            |
| `role`           | `str`                                            | `"obstacle"`     | Role of the object in the simulation.                                                    |
| `color`          | `str`                                            | `'k'` (black)    | Visualization color of the object in the simulation.                                     |
| `static`         | `bool`                                           | `False`          | Indicates if the object is static.                                                       |
| `vel_min`        | `list` of `float`                                | `[-1, -1]`       | Minimum velocity limits for each control dimension.                                      |
| `vel_max`        | `list` of `float`                                | `[1, 1]`         | Maximum velocity limits for each control dimension.                                      |
| `acce`           | `list` of `float`                                | `[inf, inf]`     | Acceleration limits.                                                                     |
| `angle_range`    | `list` of `float`                                | `[-pi, pi]`      | Range of orientation angles in radians.                                                  |
| `goal_threshold` | `float`                                          | `0.1`            | Threshold distance to determine goal arrival.                                            |
| `sensors`        | `list` of `dict`                                 | `None`           | List of sensor configurations attached to the object. Support name: `lidar2d`            |
| `arrive_mode`    | `str`                                            | `'position'`     | Mode for arrival detection.                                                              |
| `description`    | `str`                                            | `None`           | Image description or label for the object.                                               |
| `unobstructed`   | `bool`                                           | `False`          | Indicates if the object ignores collisions.                                              |
| `plot`           | `dict`                                           | `{}`             | Plotting options for object visualization.                                               |
| `state_dim`      | `int`                                            | `None`           | Dimension of the state vector.                                                           |
| `vel_dim`        | `int`                                            | `None`           | Dimension of the velocity vector.                                                        |
| `fov`            | `float`                                          | `None`           | Field of view angles in radians for the object's sensors.                                |
| `fov_radius`     | `float`                                          | `None`           | Field of view radius for the object's sensors.                                           |

### Detailed Parameter Descriptions

(object-properties)=
::::{dropdown} **object properties**

**Object Properties:**
- `number`, `distribution`, `state`, `goal`, `velocity`
- `role`, `color`, `static`, `arrive_mode`
- `vel_min`, `vel_max`, `acce`, `angle_range`

**`number`**:
  Specifies the number of objects to create using the given configuration. 

  **e.g.** 
  ```yaml
  robot:
    - number: 5
  ```

**`distribution`** ([source](https://ir-sim.readthedocs.io/en/dev/irsim.world.html#irsim.world.object_factory.ObjectFactory.generate_state_list)):
  Defines how multiple objects are spatially distributed when `number` is greater than `1`. Supported distribution types include:
  - `'manual'`: Manually specify initial states and goals for each object. 
    - In this case, the `state` (or goal) parameters must be provided for each object. If the provided list is shorter than the number of objects, the last state (or goal) is repeated.

    **e.g.** 
    ```yaml
    distribution: {name: 'manual'}
    state: [[1, 1, 0], [2, 2, 0], [3, 3, 0]]
    goal: [[9, 9, 0], [8, 8, 0], [7, 7, 0]]
    ```

  - `'random'`: Randomly distribute objects within specified ranges. Optional parameters:
    - `range_low` (list): Lower bounds for random distribution. Default is `[0, 0, -3.14]`.
    - `range_high` (list): Upper bounds for random distribution. Default is `[10, 10, 3.14]`. 

    **e.g.** 
    ```yaml
    distribution: {name: 'random', range_low: [0, 0, -3.14], range_high: [10, 10, 3.14]}
    ```

  - `'circle'`: Arrange objects in a circular formation around a specified center. Optional parameters:
    - `center` (list): Center coordinates of the circle. Default is `[5, 5, 0]`.
    - `radius` (float): Radius of the circle. Default is `4.0`.  

    **e.g.** 
    ```yaml
    distribution: {name: 'circle', center: [5, 5, 0], radius: 4.0}
    ```

**`state`**:
  Defines the initial state of the object, typically in the format `[x, y, theta]`, where `theta` represents the orientation in radians. If the provided state has more elements than required, extra elements are truncated; if fewer, missing values are filled with zeros. 

  **Example:**
  ```yaml
  state: [1.0, 1.0, 0.2]
  ```

**`velocity`**:
  Specifies the initial velocity (list) of the object. The format depends on the kinematics model:
  - For `'diff'`: `[v, omega]`, where `v` is linear velocity and `omega` is angular velocity.
  - For `'omni'`: `[vx, vy]`, velocities along the x and y axes.
  - For `'acker'`: Typically `[v, phi]`, where `v` is linear velocity and `phi` is steering angle.

  **Example:**
  ```yaml
  velocity: [1.0, 0.5]
  ```

**`goal`**:
  Sets the target state or position the object should move toward. Used in conjunction with behaviors to guide the object's navigation. The format is `[x, y, theta]` or `[[x, y, theta], [x, y, theta], ...]` for multiple goals. Default is `[10.0, 10.0, 0.0]`.

  **Example:**
  ```yaml
  goal: [10.0, 10.0, 0.2]
  ```

  or for multiple goals for the single object (Pay attention to the difference between the single goal for multiple objects and multiple goals for the single object)

  ```yaml
  goal: 
    - [[10.0, 10.0, 0.2], [5.0, 4.0, 1.0], [3.0, 3.0, 2.0]]
  ```
::::

(object-kinematics)=
::::{dropdown} **object kinematics**

**Kinematics Models:**
- `diff`: Differential drive (`[v, omega]`)
- `omni`: Omnidirectional (`[vx, vy]`)
- `acker`: Ackermann steering (`[v, phi]`)

**`kinematics`**:
  Sets the kinematic model governing the object's movement. Supported models:
  - `'diff'`: Differential drive robot, suitable for robots that can rotate in place (e.g., two-wheel robots). This type of robot is controlled by linear and angular velocity. Optional parameters:
    - `noise` (bool): whether to add noise to the velocity commands. Default is `False`.
    - `alpha` (list): noise parameters for velocity commands. Default is `[0.03, 0, 0, 0.03]`.    

    **e.g.** 
    ```yaml
    kinematics: {name: 'diff', noise: True, alpha: [0.03, 0, 0, 0.03]}
    ```

  - `'omni'`: Omnidirectional movement, allowing movement in any direction without changing orientation. This type of robot is controlled by velocities along the x and y axes. Optional parameters:
    - `noise` (bool): whether to add noise to the velocity commands. Default is False.
    - `alpha` (list): noise parameters for velocity commands. Default is `[0.03, 0, 0, 0.03]`.   

    **e.g.** 
    ```yaml
    kinematics: {name: 'omni', noise: True, alpha: [0.03, 0, 0, 0.03]}
    ```
   
  - `'acker'`: Ackermann steering, typical for car-like vehicles requiring a turning radius.
    - `noise` (bool): whether to add noise to the velocity commands. Default is False.
    - `alpha` (list): noise parameters for velocity commands. Default is `[0.03, 0, 0, 0.03]`.  
    - `mode` (str): steering mode, either `steer` or `angular`. Default is `steer`.
      - `steer`: the object is controlled by linear and steer angle.
      - `angular`: the object is controlled by linear and angular velocity. 

    **e.g.** 
    ```yaml
    kinematics: {name: 'acker', noise: True, alpha: [0.03, 0, 0, 0.03], mode: 'steer'}
    ```

  ````{warning}
    When using the `acker` kinematics model, ensure that the `wheelbase` parameter is set in the `shape` configuration.
  ````
::::

(object-shape)=
::::{dropdown} **object shape**

**Shape Types:**
- `circle`: `radius`, `center`, `random_shape`
- `rectangle`: `length`, `width`, `wheelbase`
- `polygon`: `vertices`, `is_convex`
- `linestring`: `vertices`

**`shape`**:
  Determines the geometric shape used for collision detection and visualization in the original state. Supported shapes and required parameters:

  - **`'circle'`**: Represents a circular shape.
    - **`radius`** (`float`): Radius of the circle. Default is `0.2`.
    - **`center`** (`list`): Center (x, y) of the circle. Default is `[0, 0]`.
    - **`random_shape`** (`bool`): Whether to generate a random radius. Default is `False`.
    - **`radius_range`** (`list`): Range `[min_radius, max_radius]` for random radius generation if `random_shape` is `True`. Default is `[0.1, 1.0]`.
    - **`wheelbase`** (`float`): Wheelbase of the Ackermann steering vehicle. Required when using `'acker'` kinematics. Default is `None`.

    **Example:**
    ```yaml
    shape: {name: 'circle', radius: 0.2, center: [0, 0]}
    ```

  - **`'rectangle'`**: Represents a rectangular shape.
    - **`length`** (`float`): Length of the rectangle along the x-axis. Default is `1.0`.
    - **`width`** (`float`): Width of the rectangle along the y-axis. Default is `1.0`.
    - **`wheelbase`** (`float`): Wheelbase of the Ackermann steering vehicle. Required when using `'acker'` kinematics. Default is `None`.

    **Example:**
    ```yaml
    shape: {name: 'rectangle', length: 1.0, width: 0.5}
    ```
  
  - **`'polygon'`**: Represents a polygonal shape defined by a list of vertices.
    - **`vertices`** (`list`): List of vertices defining the polygon in the format `[[x1, y1], [x2, y2], ...]`, if not provided, a random polygon will be generated.
    - **`random_shape`** (`bool`): Whether to generate a series of random polygons. Default is `False`.
    - **`is_convex`** (`bool`): Whether to generate a series of random convex polygons. Default is `False`.
    - parameters for random polygon generation, see [random_generate_polygon](#irsim.lib.algorithm.generation.random_generate_polygon) for more details. Parameters include `number `, `center_range `, `avg_radius_range `, `irregularity_range `, `spikeyness_range `, `num_vertices_range `.
      
    **Example:**
    ```yaml
    shape:
      name: 'polygon'
      vertices: 
        - [4.5, 4.5]
        - [5.5, 4.5]
        - [5.5, 5.5]
        - [4.5, 5.5]
    ```

    or 

    ```yaml
    shape:
      - {name: 'polygon', random_shape: true, center_range: [5, 10, 40, 30], avg_radius_range: [0.5, 2], irregularity_range: [0, 1], spikeyness_range: [0, 1], num_vertices_range: [4, 5]} 
    ```
  
  - **`'linestring'`**: Represents a line string shape defined by a list of vertices. Similar to a polygon but generates a line string.
    - **`vertices`** (`list`): List of vertices defining the line string in the format `[[x1, y1], [x2, y2], ...]`.
    - **`random_shape`** (`bool`): Whether to generate a series of random line strings (polygon). Default is `False`.
    - **`is_convex`** (`bool`): Whether to generate a series of random convex line strings (polygons). Default is `False`.
    - parameters for random line string generation (polygon), see [random_generate_polygon](#irsim.lib.algorithm.generation.random_generate_polygon) for more details. Parameters include `number `, `center_range `, `avg_radius_range `, `irregularity_range `, `spikeyness_range `, `num_vertices_range `.

    **Example:**
    ```yaml
    shape:
      name: 'linestring'
      vertices: 
        - [4.5, 4.5]
        - [5.5, 4.5]
        - [5.5, 5.5]
        - [4.5, 5.5]
    ```
    
    or 

    ```yaml
    shape:
      - {name: 'linestring', random_shape: true, center_range: [5, 10, 40, 30], avg_radius_range: [0.5, 2], irregularity_range: [0, 1], spikeyness_range: [0, 1], num_vertices_range: [4, 5]} 
    ```
::::

(object-behavior)=
::::{dropdown} **object behavior**

**Behavior Systems:**
- `dash`: Direct movement - `wander`, `angle_tolerance`
- `rvo`: Collision avoidance - `vxmax`, `vymax`, `acce`, `factor`

**`behavior`**:
  Configures the movement behavior of the object. Behaviors can be simple or complex and may include additional parameters. Supported behavior names:
  - `'dash'`: Moves directly toward the goal at maximum allowable speed.
    - `wander` (bool): Whether to add random wandering to the movement. If `True`, the object will have a random goal when reach current goal. Default is `False`.
    - `target_roles` (str): Only the objects with the target role will be applied to the behavior. Default is `all`. Currently, you can set the target role as `robot` or `obstacle`.
    - `range_low`(list): Lower bounds for random wandering. Default is `[0, 0, -3.14]`. 
    - `range_high`(list): Upper bounds for random wandering. Default is `[10, 10, 3.14]`.
    - `angle_tolerance` (float): Tolerance for orientation alignment with `diff` and `acker` kinematics. Default is `0.1`. 
  
    **Example:**
    ```yaml
    behavior: {name: 'dash', wander: True, range_low: [0, 0, -3.14], range_high: [10, 10, 3.14], angle_tolerance: 0.1}
    ```

  - `'rvo'`: Implements Reciprocal Velocity Obstacles for collision avoidance among multiple moving objects. Support kinematics are `diff` and `omni`.
    - `wander` (bool): Whether to add random wandering to the movement. If `True`, the object will have a random goal when reach current goal. Default is `False`.
    - `target_roles` (str): Only the objects with the target role will be applied to the behavior. Default is `all`. Currently, you can set the target role as `robot` or `obstacle`.
    - `range_low`(list): Lower bounds for random wandering. Default is `[0, 0, -3.14]`. 
    - `range_high`(list): Upper bounds for random wandering. Default is `[10, 10, 3.14]`.
    - `vxmax` (float): Maximum linear velocity in x axis. Default is `1.5`.
    - `vymax` (float): Maximum linear velocity in y axis. Default is `1.5`.
    - `acce` (float): Maximum acceleration. Default is `1.0`.
    - `factor` (float): Factor for the RVO algorithm. Default is `1.0`.
    - `mode` (str): Mode for RVO algorithm, either `rvo`, `hrvo`, or `vo`. Default is `rvo`. 
      - `rvo`: Reciprocal Velocity Obstacles. For multi-agent collision avoidance.
      - `hrvo`: Hybrid Reciprocal Velocity Obstacles. Combine RVO with VO to avoid deadlocks.
      - `vo`: Velocity Obstacles. For obstacle avoidance.
    - `neighbor_threshold` (float): Distance threshold to filter the neighbors to the self robot. Default is `3.0`.

    **Example:**
    ```yaml
    behavior: {name: 'rvo', vxmax: 1.5, vymax: 1.5, acce: 1.0, factor: 1.0, mode: 'rvo', wander: False}
    ```

**`role`**:
  Defines the object's role in the simulation, determined by the section it belongs to:
  - `'robot'`: An active entity typically controlled by behaviors or input commands.
  - `'obstacle'`: A passive entity that may or may not move but is considered during collision detection.

**`color`**:
  Specifies the object's color in visualizations for easy identification. Detailed color options can be found in [matplotlib color](https://matplotlib.org/stable/gallery/color/named_colors.html).

  **Example:**
  ```yaml
  color: 'r'
  ```

**`static`**:
  A boolean indicating whether the object is static (does not move). Static objects ignore kinematics and behaviors, remaining at their initial state.

  **Example:**
  ```yaml
  static: True
  ```
::::

(velocity-acceleration-limits)=
::::{dropdown} **Velocity & Acceleration Limits**

**`vel_min`** and **`vel_max`**:
  Set the minimum and maximum velocity limits for each control dimension (e.g., linear and angular velocities). These constraints ensure the object's motion stays within feasible and safe bounds. 

**`acce`**:
  Defines acceleration limits as the maximum change in velocity per time step for each control dimension. This parameter simulates the physical limitations of the object's motion capabilities.

**`angle_range`**:
  Specifies the allowed range of orientation angles `[min, max]` in radians. The object's orientation angle `theta` is wrapped within this range to maintain consistency. 

**`goal_threshold`**:
  Determines the proximity threshold to the goal at which the object is considered to have arrived. Once within this distance, arrival behaviors or state changes may be triggered.

  **Example:**
  ```yaml
  vel_min: [-1, -1]
  vel_max: [1, 1]
  acce: [0.5, 0.1]
  angle_range: [-pi, pi]
  goal_threshold: 0.1
  ```
::::

(object-sensor)=
::::{dropdown} **object sensors**

**Sensors:**
- `lidar2d`: `range_min`, `range_max`, `angle_range`, `number`, `noise`

**`sensors`**:
  Attaches sensors to the object for environmental perception. Each sensor is defined by a dictionary indicating its type and specific parameters. Currently supported sensor `name` (or `type`) include:
  - `lidar2d`: 2D LiDAR sensor for distance measurements. Parameters include:
    - `range_min` (float): Minimum detection range. Default is `0.0`.
    - `range_max` (float): Maximum detection range. Default is `10.0`.
    - `angle_range` (float): Total angle range of the sensor. Default is `pi`.
    - `number` (int): Number of laser beams. Default is `100`.
    - `scan_time` (float): Time taken for one complete scan. Default is `0.1`.
    - `noise` (bool): Whether noise is added to measurements. Default is `False`.
    - `std` (float): Standard deviation for range noise if `noise` is `True`. Default is `0.2`.
    - `angle_std` (float): Standard deviation for angle noise if `noise` is `True`. Default is `0.02`.
    - `offset` (list): Offset of the sensor from the object's position (x, y, theta). Default is `[0, 0, 0]`.
    - `alpha` (float): Transparency for plotting. Default is `0.3`.
    - `has_velocity` (bool): Whether measures the lidar point velocity. Default is `False`.
    - `color` (str): Color of the sensor. Default is `r`.

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

**`arrive_mode`**:
  Chooses the method for determining if the object has arrived at its goal:
  - `'position'`: Arrival is based solely on proximity to the goal position (`[x, y]`).
  - `'state'`: Considers both position and orientation in the arrival check (`[x, y, theta]`).

   **Example:**
    ```yaml
    arrive_mode: 'position'
    ```

**`description`**:
  Provides a image for representing the object graphically. Supports image file located in world/description. You can also set the absolute path of the image file by your need.

  - `car_green.png`: A default image for the ackermann steering vehicle.
  - `car_blue.png`
  - `car_red.png`
  - `diff_robot0.png`
  - `diff_robot1.png`
  
  **Example:**
  ```yaml
  description: 'car_blue.png'
  ```

**`unobstructed`**:
  When set to `True`, this object is treated as having an unobstructed path, ignoring collisions with other objects and obstacles. This can be useful for testing or for objects that must not be impeded.

**`fov`** and **`fov_radius`**:
  Define the field of view (FOV) for the object's sensors. The FOV is the angular range within which the sensor can detect objects. The `fov` parameter specifies the angular range in radians, while `fov_radius` sets the maximum detection distance.

  **Example:**
  ```yaml
  fov: 1.57
  fov_radius: 5.0
  ```

**`state_dim`** and **`vel_dim`**:
  Specify the dimensions of the state and velocity vectors. These are typically inferred from the kinematics model but can be explicitly set if needed.

  **Example:**
  ```yaml
  state_dim: 3
  vel_dim: 2
  ```
::::

(object-visualization)=
::::{dropdown} **object visualization**

**Visualization:**
- Object: `obj_color`, `obj_alpha`, `obj_linestyle`
- Goal: `show_goal`, `goal_color`, `goal_alpha`
- Trail: `show_trail`, `trail_freq`, `trail_type`
- Sensor: `show_sensor`, `show_fov`, `fov_color`

**`plot`**:
  Contains plotting options controlling the visual representation of the object. All plot elements are initially created at the origin and positioned using transforms and data updates during animation.

  **Object Visualization Properties:**
  - `obj_linestyle` (str): Line style for object outline (e.g., '-', '--', ':', '-.'). Default is '-'.
  - `obj_zorder` (int): Z-order (drawing layer) for object elements. Default is 3 for robots, 1 for obstacles.
  - `obj_color` (str): Color of the object. Default is the object's color property.
  - `obj_alpha` (float): Transparency of the object (0.0 to 1.0). Default is 1.0.
  - `obj_linewidth` (float): Width of the object outline. Default varies by object type.

  **Goal Visualization:**
  - `show_goal` (bool): Whether to show the goal position. Default is False.
    - `goal_color` (str): Color of the goal marker. Default is the object's color.
    - `goal_alpha` (float): Transparency of the goal marker (0.0 to 1.0). Default is 0.5.
    - `goal_zorder` (int): Z-order of the goal marker. Default is 1.

  **Text Label Visualization:**
  - `show_text` (bool): Whether to show text information. Default is False.
    - `text_color` (str): Color of the text. Default is 'k' (black).
    - `text_size` (int): Font size of the text. Default is 10.
    - `text_alpha` (float): Transparency of the text (0.0 to 1.0). Default is 1.0.
    - `text_zorder` (int): Z-order of the text. Default is 2.
    - `text_position` (list): Position offset from object center [dx, dy]. Default is [-radius-0.1, radius+0.1].

  **Velocity Arrow Visualization:**
  - `show_arrow` (bool): Whether to show the velocity arrow. Default is False.
    - `arrow_color` (str): Color of the arrow. Default is "gold".
    - `arrow_length` (float): Length of the arrow. Default is 0.4.
    - `arrow_width` (float): Width of the arrow. Default is 0.6.
    - `arrow_alpha` (float): Transparency of the arrow (0.0 to 1.0). Default is 1.0.
    - `arrow_zorder` (int): Z-order of the arrow. Default is 4.

  **Trajectory Path Visualization:**
  - `show_trajectory` (bool): Whether to show the trajectory line. Default is False.
    - `traj_color` (str): Color of the trajectory. Default is the object's color.
    - `traj_style` (str): Line style of the trajectory (e.g., '-', '--', ':', '-.'). Default is "-".
    - `traj_width` (float): Width of the trajectory line. Default is the object's width.
    - `traj_alpha` (float): Transparency of the trajectory (0.0 to 1.0). Default is 0.5.
    - `traj_zorder` (int): Z-order for trajectory elements. Default is 0.

  **Object Trail Visualization:**
  - `show_trail` (bool): Whether to show object trails. Default is False.
    - `trail_freq` (int): Frequency of trail display (every N steps). Default is 2.
    - `trail_type` (str): Type of trail shape. Default is the object's shape.
    - `trail_edgecolor` (str): Edge color of the trail. Default is the object's color.
    - `trail_linewidth` (float): Width of the trail outline. Default is 0.8.
    - `trail_alpha` (float): Transparency of the trail (0.0 to 1.0). Default is 0.7.
    - `trail_fill` (bool): Whether to fill the trail shape. Default is False.
    - `trail_color` (str): Fill color of the trail. Default is the object's color.
    - `trail_zorder` (int): Z-order for trail elements. Default is 0.

  **Sensor Visualization:**
  - `show_sensor` (bool): Whether to show sensor visualizations. Default is True.

  **Field of View Visualization:**
  - `show_fov` (bool): Whether to show field of view visualization. Default is False.
    - `fov_color` (str): Fill color of the field of view. Default is "lightblue".
    - `fov_edge_color` (str): Edge color of the field of view. Default is "blue".
    - `fov_alpha` (float): Transparency of the field of view (0.0 to 1.0). Default is 0.5.
    - `fov_zorder` (int): Z-order of the field of view. Default is 1.

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








