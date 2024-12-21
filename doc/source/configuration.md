YAML Configuration Syntax
==================

The configuration file is a YAML file to initialize the environment. It contains the parameters of the world, obstacle, and robot. You can customize the simulation environment by modifying the parameters in the configuration file. 

The configuration file is divided into three main sections: `world`, `robot`, and `obstacle`. Following is a simple example of the configuration file:

```yaml
world:
  height: 10  # the height of the world
  width: 10   # the height of the world
  step_time: 0.1  # 10Hz calculate each step
  sample_time: 0.1  # 10 Hz for render and data extraction 
  offset: [0, 0] # the offset of the world on x and y 
  collision_mode: 'stop'  # 'stop', 'unobstructed', 'reactive'

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

:::{Note}
  - To include several robots or obstacles in the configuration file, add separate entries under the robot and obstacle sections using `-` for each additional item.
  - Parameters such as distribution, shape, behavior, and kinematics must be formatted as `{key: value}` pairs. Ensure that each dictionary includes the `name` key; omitting name will result in a None value for that parameter.
  - When dealing with multiple objects (i.e., when the number is greater than 1), utilize the `distribution` parameter to define how these objects are distributed.
  - By default, all objects within the same group share identical configurations. To customize individual objects within a group, add sub-parameters using `-`. Any additional objects not explicitly configured will inherit the settings of the last specified object in the group.
:::

## World Configuration

The `world` section contains the configuration of the simulation environment. The following table details the configuration parameters for the world:

| **Parameter**    | **Type**          | **Default** | **Description**                                                                                |
| ---------------- | ----------------- | ----------- | ---------------------------------------------------------------------------------------------- |
| `name`           | `str`             | `"world"`   | Name of the world                                                                              |
| `height`         | `float`           | `10`        | Height of the world (meter)                                                                    |
| `width`          | `float`           | `10`        | Width of the world (meter)                                                                     |
| `step_time`      | `float`           | `0.1`       | Time interval between simulation steps (in seconds)                                            |
| `sample_time`    | `float`           | `0.1`       | Time interval between samples for rendering and data extraction (in seconds)                   |
| `offset`         | `list` of `float` | `[0, 0]`    | Offset for the world's position in `[x, y]` coordinates                                        |
| `control_mode`   | `str`             | `"auto"`    | Control mode of the simulation. Support mode: `auto` or `keyboard`                             |
| `collision_mode` | `str`             | `"stop"`    | Collision handling mode (Support: `"stop"`, `"reactive"`, `"unobstructed"`)                    |
| `obstacle_map`   | `str` (file path) | `None`      | Path to the image file representing the obstacle map                                           |
| `mdownsample`    | `int`             | `1`         | Downsampling factor for the obstacle map to reduce resolution and decrease computational load. |


### Detailed Description of Parameters

- **`name`**:  
  Defines the name of the world used in the simulation. This can be useful for identifying different simulation environments.

- **`height`**:  
  Specifies the vertical size of the world in units of meters in the Y-axis direction plotted on the screen.

- **`width`**:  
  Specifies the horizontal size of the world in units of of meters in the X-axis direction plotted on the screen.

- **`step_time`**:  
  Determines the time interval between each simulation step. A smaller `step_time` results in a higher simulation frequency (e.g., `0.1` seconds corresponds to 10 Hz) but need longer time to run the simulation.

- **`sample_time`**:  
  Defines the time interval for rendering the simulation and extracting data. This controls how frequently visual updates and data recordings occur.

- **`offset`**:  
  Sets the initial positional offset of the world on the X and Y axes. This is useful for positioning the world within a larger coordinate system or for relative placement.

- **`control_mode`**:  
  Configures how the objects in the simulation is controlled:
  - `auto`: Automatic control by the input velocities defined in python script or behavior in the YAML file.
  - `keyboard`: Manual control via keyboard inputs. The key inputs are defined in the file.

- **`collision_mode`**:  
  Defines how collisions between objects are handled in the simulation:
  - `stop`: Stops the movement of objects upon collision.
  - `reactive`: Objects react to collisions based on predefined behaviors. 
  - `unobstructed`: Allows objects to pass through each other without consideration of any collision.

- **`obstacle_map`**:  
  Specifies the file path to an image that serves as the obstacle map. This image is used to generate the grid map that defines the positions of obstacles within the world. Each pixel in the image corresponds to a grid cell in the map, where the color of the pixel determines the presence of an obstacle. 

- **`mdownsample`**:  
  Sets the downsampling factor for the obstacle map image. A higher value reduces the resolution of the obstacle map, which can optimize the simulation performance by decreasing computational load. 

### Complete Example of World Configuration

Below is a comprehensive example of the `world` section in the YAML configuration file:

```yaml
world:
  name: "world"                       # Name of the world
  height: 10                          # Height of the world
  width: 10                           # Width of the world
  step_time: 0.1                      # Time interval between steps (10 Hz)
  sample_time: 0.1                    # Time interval for rendering and data extraction (10 Hz)
  offset: [0, 0]                      # Positional offset of the world on the x and y axes
  control_mode: 'auto'                # Control mode ('auto' or 'keyboard')
  collision_mode: 'stop'              # Collision handling mode ('stop', 'unobstructed', 'reactive')
  obstacle_map: "path/to/map.png"     # Path to the obstacle map image file
  mdownsample: 2                      # Downsampling factor for the obstacle map
```

:::{warning}
**`obstacle_map`**: Replace `"path/to/map.png"` with the actual file path to your obstacle map image. Ensure that the image is in a compatible format (e.g., PNG, JPEG) and properly represents obstacle locations.
:::


## Object Configuration (Robot and Obstacle)

All `robot` and `obstacle` entities in the simulation are configured as objects with similar parameters but may have different default values. This section outlines the configuration parameters available for these objects.

| Parameter        | Type              | Default          | Description                                                                              |
| ---------------- | ----------------- | ---------------- | ---------------------------------------------------------------------------------------- |
| `number`         | `int`             | `1`              | Number of objects to create.                                                             |
| `distribution`   | `dict`            | `{name: manual}` | Defines how multiple objects are distributed. Support name: `manual`, `random`, `circle` |
| `kinematics`     | `dict`            | `None`           | Kinematic model of the object. Support name: `diff`, `acker`, `omni`                     |
| `shape`          | `dict`            | `{name: circle}` | Shape of the object.  Support name:  `circle`, `rectangle`, `polygon` , `linestring`     |
| `state`          | `list` of `float` | `[0, 0, 0]`      | Initial state vector of the object.                                                      |
| `velocity`       | `list` of `float` | `[0, 0]`         | Initial velocity vector.                                                                 |
| `goal`           | `list` of `float` | `[10, 10, 0]`    | Goal state vector.                                                                       |
| `behavior`       | `dict`            | `{name: dash}`   | Behavior configuration dictating object movement. Support name: `dash`, `rvo`            |
| `role`           | `str`             | `Obstacle`       | Role of the object in the simulation.                                                    |
| `color`          | `str`             | `'k'` (black)    | Visualization color of the object in the simulation.                                     |
| `static`         | `bool`            | `False`          | Indicates if the object is static.                                                       |
| `vel_min`        | `list` of `float` | `[-1, -1]`       | Minimum velocity limits for each control dimension.                                      |
| `vel_max`        | `list` of `float` | `[1, 1]`         | Maximum velocity limits for each control dimension.                                      |
| `acce`           | `list` of `float` | `[inf, inf]`     | Acceleration limits.                                                                     |
| `angle_range`    | `list` of `float` | `[-pi, pi]`      | Range of orientation angles in radians.                                                  |
| `goal_threshold` | `float`           | `0.1`            | Threshold distance to determine goal arrival.                                            |
| `sensors`        | `list` of `dict`  | `None`           | List of sensor configurations attached to the object. Support name: `lidar2d`            |
| `arrive_mode`    | `str`             | `'position'`     | Mode for arrival detection.                                                              |
| `description`    | `str`             | `None`           | Image description or label for the object.                                               |
| `unobstructed`   | `bool`            | `False`          | Indicates if the object ignores collisions.                                              |
| `plot`           | `dict`            | `{}`             | Plotting options for object visualization.                                               |
| `state_dim`      | `int`             | `None`           | Dimension of the state vector.                                                           |
| `vel_dim`        | `int`             | `None`           | Dimension of the velocity vector.                                                        |


### Detailed Description of Parameters
----

- **`number`**:
  Specifies the number of objects to create using the given configuration. 

  **e.g.** 
  ```yaml
  robot:
    - number: 5
  ```

- **`distribution`** ([source](https://ir-sim.readthedocs.io/en/dev/irsim.world.html#irsim.world.object_factory.ObjectFactory.generate_state_list)):
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
  
- **`kinematics`**:
  Sets the kinematic model governing the object's movement. Supported models:
  - `'diff'`: Differential drive robot, suitable for robots that can rotate in place (e.g., two-wheel robots). Optional parameters:
    - `noise` (bool): whether to add noise to the velocity commands. Default is `False`.
    - `alpha` (list): noise parameters for velocity commands. Default is `[0.03, 0, 0, 0.03]`.    

    **e.g.** 
    ```yaml
    kinematics: {name: 'diff', noise: True, alpha: [0.03, 0, 0, 0.03]}
    ```

  - `'omni'`: Omnidirectional movement, allowing movement in any direction without changing orientation.
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

  :::{warning}
    When using the `acker` kinematics model, ensure that the `wheelbase` parameter is set in the `shape` configuration.
  :::

- **`shape`**:
  Determines the geometric shape used for collision detection and visualization in the original state. Supported shapes and required parameters:

  - **`'circle'`**: Represents a circular shape.
    - **`radius`** (`float`): Radius of the circle. Default is `0.2`.
    - **`random_shape`** (`bool`): Whether to generate a random radius. Default is `False`.
    - **`radius_range`** (`list`): Range `[min_radius, max_radius]` for random radius generation if `random_shape` is `True`. Default is `[0.1, 1.0]`.
    - **`wheelbase`** (`float`): Wheelbase of the Ackermann steering vehicle. Required when using `'acker'` kinematics. Default is `None`.

    **Example:**
    ```yaml
    shape: {name: 'circle', radius: 0.2}
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

- **`state`**:
  Defines the initial state of the object, typically in the format `[x, y, theta]`, where `theta` represents the orientation in radians. If the provided state has more elements than required, extra elements are truncated; if fewer, missing values are filled with zeros. 

  **Example:**
  ```yaml
  state: [1.0, 1.0, 0.2]
  ```

- **`velocity`**:
  Specifies the initial velocity (list) of the object. The format depends on the kinematics model:
  - For `'diff'`: `[v, omega]`, where `v` is linear velocity and `omega` is angular velocity.
  - For `'omni'`: `[vx, vy]`, velocities along the x and y axes.
  - For `'acker'`: Typically `[v, phi]`, where `v` is linear velocity and `phi` is steering angle.

  **Example:**
  ```yaml
  velocity: [1.0, 0.5]
  ```

- **`goal`**:
  Sets the target state or position the object should move toward. Used in conjunction with behaviors to guide the object's navigation. The format is `[x, y, theta]`. Default is `[10.0, 10.0, 0.0]`.

  **Example:**
  ```yaml
  goal: [10.0, 10.0, 0.2]
  ```

- **`behavior`**:
  Configures the movement behavior of the object. Behaviors can be simple or complex and may include additional parameters. Supported behavior names:
  - `'dash'`: Moves directly toward the goal at maximum allowable speed.
    - `wander` (bool): Whether to add random wandering to the movement. If `True`, the object will have a random goal when reach current goal. Default is `False`.
    - `range_low`(list): Lower bounds for random wandering. Default is `[0, 0, -3.14]`. 
    - `range_high`(list): Upper bounds for random wandering. Default is `[10, 10, 3.14]`.
    - `angle_tolerance` (float): Tolerance for orientation alignment with `diff` and `acker` kinematics. Default is `0.1`. 
  
    **Example:**
    ```yaml
    behavior: {name: 'dash', wander: True, range_low: [0, 0, -3.14], range_high: [10, 10, 3.14], angle_tolerance: 0.1}
    ```

  - `'rvo'`: Implements Reciprocal Velocity Obstacles for collision avoidance among multiple moving objects. Support kinematics are `diff` and `omni`.
    - `wander` (bool): Whether to add random wandering to the movement. If `True`, the object will have a random goal when reach current goal. Default is `False`.
    - `range_low`(list): Lower bounds for random wandering. Default is `[0, 0, -3.14]`. 
    - `range_high`(list): Upper bounds for random wandering. Default is `[10, 10, 3.14]`.
    - `vxmax` (float): Maximum linear velocity in x axis. Default is `1.5`.
    - `vymax` (float): Maximum linear velocity in y axis. Default is `1.5`.
    - `acceler` (float): Maximum acceleration. Default is `1.0`.
    - `factor` (float): Factor for the RVO algorithm. Default is `1.0`.
    - `mode` (str): Mode for RVO algorithm, either `rvo`, `hrvo`, or `vo`. Default is `rvo`. 
      - `rvo`: Reciprocal Velocity Obstacles. For multi-agent collision avoidance.
      - `hrvo`: Hybrid Reciprocal Velocity Obstacles. Combine RVO with VO to avoid deadlocks.
      - `vo`: Velocity Obstacles. For obstacle avoidance.

    **Example:**
    ```yaml
    behavior: {name: 'rvo', vxmax: 1.5, vymax: 1.5, acceler: 1.0, factor: 1.0, mode: 'rvo', wander: False}
    ```

- **`role`**:
  Defines the object's role in the simulation, determined by the section it belongs to:
  - `'robot'`: An active entity typically controlled by behaviors or input commands.
  - `'obstacle'`: A passive entity that may or may not move but is considered during collision detection.

- **`color`**:
  Specifies the object's color in visualizations for easy identification. Detailed color options can be found in [matplotlib color](https://matplotlib.org/stable/gallery/color/named_colors.html).

  **Example:**
  ```yaml
  color: 'r'
  ```

- **`static`**:
  A boolean indicating whether the object is static (does not move). Static objects ignore kinematics and behaviors, remaining at their initial state.

  **Example:**
  ```yaml
  static: True
  ```

- **`vel_min`** and **`vel_max`**:
  Set the minimum and maximum velocity limits for each control dimension (e.g., linear and angular velocities). These constraints ensure the object's motion stays within feasible and safe bounds. 

- **`acce`**:
  Defines acceleration limits as the maximum change in velocity per time step for each control dimension. This parameter simulates the physical limitations of the object's motion capabilities.

- **`angle_range`**:
  Specifies the allowed range of orientation angles `[min, max]` in radians. The object's orientation angle `theta` is wrapped within this range to maintain consistency. 

- **`goal_threshold`**:
  Determines the proximity threshold to the goal at which the object is considered to have arrived. Once within this distance, arrival behaviors or state changes may be triggered.

  **Example:**
  ```yaml
  vel_min: [-1, -1]
  vel_max: [1, 1]
  acce: [0.5, 0.1]
  angle_range: [-pi, pi]
  goal_threshold: 0.1
  ```

- **`sensors`**:
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

- **`arrive_mode`**:
  Chooses the method for determining if the object has arrived at its goal:
  - `'position'`: Arrival is based solely on proximity to the goal position (`[x, y]`).
  - `'state'`: Considers both position and orientation in the arrival check (`[x, y, theta]`).

   **Example:**
    ```yaml
    arrive_mode: 'position'
    ```

- **`description`**:
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

- **`unobstructed`**:
  When set to `True`, this object is treated as having an unobstructed path, ignoring collisions with other objects and obstacles. This can be useful for testing or for objects that must not be impeded.

- **`plot`**:
  Contains plotting options controlling the visual representation of the object. Supported options include:
  - `'show_goal'`(bool): Whether to display the object's goal in visualizations. Default is `False`. 
  - `'show_arrow'`: Show arrows indicating velocity or heading direction. Default is `False`. If `True`, the following parameters can be set:
    - `arrow_length` (float): Length of the arrow. Default is `0.4`.
    - `arrow_width` (float): Width of the arrow. Default is `0.6`.
    - `arrow_color` (str): Color of the arrow. Default is `'gold'`.
  - `'show_trajectory'`: Plot the path the object has taken over time. Default is `False`. If `True`, the following parameters can be set:
    - `traj_color` (str): Color of the trajectory. Default is the object's color.
    - `traj_style` (str): Line style of the trajectory. Default is `'-'`.
    - `traj_width` (float): Line width of the trajectory. Default is the object's width.
    - `traj_alpha` (float): Transparency of the trajectory. Default is `0.5`.
  - `show_trail` (bool): Show the trail of the object. Default is `False`. If `True`, the following parameters can be set:
    - `trail_type` (str): Type of trail shape. Default is the object's shape.
    - `trail_edgecolor` (str): Edge color of the trail. Default is the object's color.
    - `trail_linewidth` (float): Line width of the trail. Default is `0.8`.
    - `trail_alpha` (float): Transparency of the trail. Default is `0.7`.
    - `trail_fill` (bool): Whether to fill the trail shape. Default is `False`.
    - `trail_color` (str): Color of the trail. Default is the object's color.
    - `trail_freq` (int): Frequency of trail updates. Default is `2`.
  - `show_sensor` (bool): Display the sensor in the visualization. Default is `True`.
  - `goal_color` (str): Color of the goal marker. Default is the object's color.

  **Example:**
  ```yaml

  plot:
    show_goal: True
    show_arrow: True
    arrow_length: 0.4
    arrow_width: 0.6
    arrow_color: 'gold'
    show_trajectory: True
    traj_color: 'blue'
    traj_style: '-'
    traj_width: 0.6
    traj_alpha: 0.5
    show_trail: True
    trail_type: 'circle'
    trail_edgecolor: 'red'
    trail_linewidth: 0.8
    trail_alpha: 0.7
    trail_fill: False
    trail_color: 'red'
    trail_freq: 2
    show_sensor: True
    goal_color: 'red'
  ```
  
- **`state_dim`** and **`vel_dim`**:
  Specify the dimensions of the state and velocity vectors. These are typically inferred from the kinematics model but can be explicitly set if needed.

  **Example:**
  ```yaml

  state_dim: 3
  vel_dim: 2
  ```

### Example Object Configurations

:::{tip}
Let's give readers a helpful hint!
:::

#### Example 1: Configuring Multiple Robots with RVO Behavior

```yaml
robot:
  - number: 10
    distribution: {name: 'circle', radius: 4.0, center: [5, 5]}  
    kinematics: {name: 'diff'}
    shape: 
      - {name: 'circle', radius: 0.2}  
    behavior: {name: 'rvo', vxmax: 1.5, vymax: 1.5, accer: 1.0, factor: 1.0}
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

#### Example 2: Configuring Various Obstacles

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

#### Example 3: Configuring an Ackermann Steering Vehicle

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

:::{Note}
- **Multiple Objects**: When configuring multiple objects, use the `number` and `distribution` parameters to efficiently generate them. For instance, setting `number: 10` with a `distribution` of `'random'` can quickly populate the simulation with randomly placed objects. 
- **Dictionary Parameters**: All dictionary-type parameters (e.g., `distribution`, `shape`, `kinematics`, `behavior`) must include a `'name'` key to specify their type. Omitting the `'name'` key will result in default values or errors.
- **Group Configurations**: By default, objects within the same group share configurations. To customize individual objects within a group, add sub-parameters using `-`. Unspecified objects will inherit the last defined configuration within the group.
- **Kinematics and Velocities**: Ensure that the `velocity` and `vel_max` parameters match the kinematics model. For example, a differential drive robot (`'diff'`) should have velocities in `[v, omega]`, while an omnidirectional robot (`'omni'`) uses `[vx, vy]`.
- **Plotting Options**: Customize the visualization of your simulation through the `plot` parameter for each object if the `plot` section is located in the object configuration. If it is located in the root of the object configuration, it will be applied to all objects. 
:::

By carefully configuring these parameters, you can create a rich and dynamic simulation environment tailored to your specific needs.






