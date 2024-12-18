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

**Note**:
  - To include several robots or obstacles in the configuration file, add separate entries under the robot and obstacle sections using `-` for each additional item.
  - Parameters such as distribution, shape, behavior, and kinematics must be formatted as `{key: value}` pairs. Ensure that each dictionary includes the `name` key; omitting name will result in a None value for that parameter.
  - When dealing with multiple objects (i.e., when the number is greater than 1), utilize the `distribution` parameter to define how these objects are distributed.
  - By default, all objects within the same group share identical configurations. To customize individual objects within a group, add sub-parameters using `-`. Any additional objects not explicitly configured will inherit the settings of the last specified object in the group.

## World Configuration

The `world` section contains the configuration of the simulation environment. The following table details the configuration parameters for the world:

| Parameter        | Description                                                                                    | Type              | Default   |
| ---------------- | ---------------------------------------------------------------------------------------------- | ----------------- | --------- |
| `name`           | Name of the world                                                                              | `str`             | `"world"` |
| `height`         | Height of the world (meter)                                                                    | `float`           | `10`      |
| `width`          | Width of the world  (meter)                                                                    | `float`           | `10`      |
| `step_time`      | Time interval between simulation steps (in seconds)                                            | `float`           | `0.1`     |
| `sample_time`    | Time interval between samples for rendering and data extraction (in seconds)                   | `float`           | `0.1`     |
| `offset`         | Offset for the world's position in `[x, y]` coordinates                                        | `list` of `float` | `[0, 0]`  |
| `control_mode`   | Control mode of the simulation (`"auto"` or `"keyboard"`)                                      | `str`             | `"auto"`  |
| `collision_mode` | Collision handling mode (`"stop"`, `"reactive"`, `"unobstructed"`)                             | `str`             | `"stop"`  |
| `obstacle_map`   | Path to the image file representing the obstacle map                                           | `str` (file path) | `None`    |
| `mdownsample`    | Downsampling factor for the obstacle map to reduce resolution and decrease computational load. | `int`             | `1`       |

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
  - `"auto"`: Automatic control by the input velocities defined in python script or behavior in the YAML file.
  - `"keyboard"`: Manual control via keyboard inputs. The key inputs are defined in the file.

- **`collision_mode`**:  
  Defines how collisions between objects are handled in the simulation:
  - `"stop"`: Stops the movement of objects upon collision.
  - `"reactive"`: Objects react to collisions based on predefined behaviors. 
  - `"unobstructed"`: Allows objects to pass through each other without consideration of any collision.

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
  obstacle_map: "path/to/map.png"      # Path to the obstacle map image file
  mdownsample: 2                      # Downsampling factor for the obstacle map
```

**Notes:**

- **`obstacle_map`**: Replace `"path/to/map.png"` with the actual file path to your obstacle map image. Ensure that the image is in a compatible format (e.g., PNG, JPEG) and properly represents obstacle locations.

<!-- ## Object Configuration (Robot and Obstacle Configuration)

All `robot` and `obstacle` entities in the simulation are configured as objects with similar parameters but may have different default values. This section outlines the configuration parameters available for these objects.

| Parameter      | Description                                                                                                   | Type   | Default            |
| -------------- | ------------------------------------------------------------------------------------------------------------- | ------ | ------------------ |
| `number`       | number of the configured objects                                                                              | `int`  | `1`                |
| `distribution` | distribution of the multiple objects, support dictionary values are `manual`, `circle`, and  `random`.        | `int`  | `{name: 'manual'}` |
| `kinematics`   | Defines the kinematics model of the object. Support name values in Dictionary are `diff`, `omni`, `acker`.    | `dict` | `None`             |
| `shape`        | Defines the shape of the object. Support name values in Dictionary are `circle`, `rectangle`, `polygon`, etc. | `dict` | `{name: 'circle'}` |
| `state`        | Initial state of the object in the format `[x, y, theta]`                                                     | `list` | `[0, 0, 0]`        |
| `velocity`     | Initial velocity of the object                                                                                | `list` | `[0, 0]`           |

| `behavior`     | Defines the behavior of the object. Possible values are `dash`, `rvo`, etc.                                | `dict` | `{'name': 'dash'}`                  | 
| `color`        | Color of the object in the simulation                                                                      | `str`  | `'g'`                               | 
| `plot`         | Plotting options for the object                                                                            | `dict` | `{'show_trajectory': True}`         | 
| `sensors`      | List of sensors attached to the object                                                                     | `list` | `None`                              |
| `distribution` | Distribution of multiple objects                                                                           | `dict` | `None`                              | 
| `unobstructed` | Flag to indicate if the object is unobstructed by other objects                                            | `bool` | `False`                             |  -->


## Object Configuration (Robot and Obstacle)

All `robot` and `obstacle` entities in the simulation are configured as objects with similar parameters but may have different default values. This section outlines the configuration parameters available for these objects.

### Parameters

| Parameter        | Description                                           | Type              | Default            |
| ---------------- | ----------------------------------------------------- | ----------------- | ------------------ |
| `number`         | Number of objects to create                           | `int`             | `1`                |
| `distribution`   | Defines how multiple objects are distributed          | `dict`            | `{name: 'manual'}` |
| `kinematics`     | Kinematic model of the object.                        | `dict`            | `None`             |
| `shape`          | Shape of the object.                                  | `dict`            | `{name: 'circle'}` |
| `state`          | Initial state vector of the object.                   | `list` of `float` | `[0, 0, 0]`        |
| `velocity`       | Initial velocity vector.                              | `list` of `float` | `[0, 0]`           |
| `goal`           | Goal state vector.                                    | `list` of `float` | `[10, 10, 0]`      |
| `behavior`       | Behavior configuration dictating object movement.     | `dict`            | `None`             |
| `role`           | Role of the object in the simulation.                 | `str`             | `Obstacle`         |
| `color`          | Visualization color of the object in the simulation.  | `str`             | `'k'` (black)      |
| `static`         | Indicates if the object is static                     | `bool`            | `False`            |
| `vel_min`        | Minimum velocity limits for each control dimension.   | `list` of `float` | `[-1, -1]`         |
| `vel_max`        | Maximum velocity limits for each control dimension.   | `list` of `float` | `[1, 1]`           |
| `acce`           | Acceleration limits.                                  | `list` of `float` | `[inf, inf]`       |
| `angle_range`    | Range of orientation angles in radians.               | `list` of `float` | `[-pi, pi]`        |
| `goal_threshold` | Threshold distance to determine goal arrive.          | `float`           | `0.1`              |
| `sensors`        | List of sensor configurations attached to the object. | `list` of `dict`  | `None`             |
| `arrive_mode`    | Mode for arrival detection.                           | `str`             | `'position'`       |
| `description`    | Image Description or label for the object.            | `str`             | `None`             |
| `unobstructed`   | if the object ignores collisions.                     | `bool`            | `False`            |
| `plot`           | Plotting options for Object visualization.            | `dict`            | `{}`               |
| `state_dim`      | Dimension of the state vector.                        | `int`             | `None`             |
| `vel_dim`        | Dimension of the velocity vector.                     | `int`             | `None`             |

### Detailed Description of Parameters
----

- **`number`**:
  Specifies the number of objects to create using the given configuration. 
<br/>

- **`distribution`** ([source](https://ir-sim.readthedocs.io/en/dev/irsim.world.html#irsim.world.object_factory.ObjectFactory.generate_state_list)):
  Defines how multiple objects are spatially distributed when `number` is greater than `1`. Supported distribution types include:
  - `'manual'`: Manually specify initial states and goals for each object. 
    - In this case, the `state` (or goal) parameters must be provided for each object. If the provided list is shorter than the number of objects, the last state (or goal) is repeated. 
    ~ **e.g.** `{name: 'manual'}`.
  - `'random'`: Randomly distribute objects within specified ranges. Optional parameters:
    - `range_low` (list): Lower bounds for random distribution. Default is `[0, 0, -3.14]`.
    - `range_high` (list): Upper bounds for random distribution. Default is `[10, 10, 3.14]`.
    ~ **e.g.** `{name: 'random', range_low: [0, 0, -3.14], range_high: [30, 10, 3.14]}`.
  - `'circle'`: Arrange objects in a circular formation around a specified center. Optional parameters:
    - `center` (list): Center coordinates of the circle. Default is `[5, 5, 0]`.
    - `radius` (float): Radius of the circle. Default is `4.0`.
    ~ **e.g.** `{name: 'circle', center: [2, 5, 0], radius: 5.0}`.
<br/>

- **`kinematics`([source]())**:
  Sets the kinematic model governing the object's movement. Supported models:
  - `'diff'`: Differential drive robot, suitable for robots that can rotate in place (e.g., two-wheel robots). Optional parameters:
    - `noise` (bool): whether to add noise to the velocity commands. Default is False.
    - `alpha` (list): noise parameters for velocity commands. Default is `[0.03, 0, 0, 0.03]`.
    ~**e.g.** `{name: 'diff', noise: True, alpha: [0.03, 0, 0, 0.03]}`.
  - `'omni'`: Omnidirectional movement, allowing movement in any direction without changing orientation.
  - `'acker'`: Ackermann steering, typical for car-like vehicles requiring a turning radius.

  Additional parameters may be required, such as `wheelbase` for the `'acker'` model, denoting the distance between the front and rear axles.

- **`wheelbase`**:
  Specifies the distance between the front and rear wheels for objects using the `'acker'` kinematics model. This parameter affects the turning radius and handling of the vehicle.

- **`shape`**:
  Determines the geometric shape used for collision detection and visualization. Supported shapes and required parameters:
  - `'circle'`: Requires `radius`.
  - `'rectangle'`: Requires `length` and `width`.
  - `'polygon'`: Requires a list of `vertices` defining the polygon.

- **`state`**:
  Defines the initial state of the object, typically in the format `[x, y, theta]`, where `theta` represents the orientation in radians. If the provided state has more elements than required, extra elements are truncated; if fewer, missing values are filled with zeros.

- **`velocity`**:
  Specifies the initial velocity of the object. The format depends on the kinematics model:
  - For `'diff'`: `[v, omega]`, where `v` is linear velocity and `omega` is angular velocity.
  - For `'omni'`: `[vx, vy]`, velocities along the x and y axes.
  - For `'acker'`: Typically `[v, phi]`, where `v` is linear velocity and `phi` is steering angle.

- **`goal`**:
  Sets the target state or position the object should move toward. Used in conjunction with behaviors to guide the object's navigation.

- **`behavior`**:
  Configures the movement behavior of the object. Behaviors can be simple or complex and may include additional parameters. Supported behaviors:
  - `'dash'`: Moves directly toward the goal at maximum allowable speed.
  - `'rvo'`: Implements Reciprocal Velocity Obstacles for collision avoidance among multiple moving objects.

  Behaviors can be specified as a string (name) or a dictionary with parameters (e.g., `{name: 'rvo', neighbor_distance: 5}`).

- **`role`**:
  Defines the object's role in the simulation:
  - `'robot'`: An active entity typically controlled by behaviors or input commands.
  - `'obstacle'`: A passive entity that may or may not move but is considered during collision detection.

- **`color`**:
  Specifies the object's color in visualizations for easy identification.

- **`static`**:
  A boolean indicating whether the object is static (does not move). Static objects ignore kinematics and behaviors, remaining at their initial state.

- **`vel_min`** and **`vel_max`**:
  Set the minimum and maximum velocity limits for each control dimension (e.g., linear and angular velocities). These constraints ensure the object's motion stays within feasible and safe bounds.

- **`acce`**:
  Defines acceleration limits as the maximum change in velocity per time step for each control dimension. This parameter simulates the physical limitations of the object's motion capabilities.

- **`angle_range`**:
  Specifies the allowed range of orientation angles `[min, max]` in radians. The object's orientation angle `theta` is wrapped within this range to maintain consistency.

- **`goal_threshold`**:
  Determines the proximity threshold to the goal at which the object is considered to have arrived. Once within this distance, arrival behaviors or state changes may be triggered.

- **`sensors`**:
  Attaches sensors to the object for environmental perception. Each sensor is defined by a dictionary indicating its type and specific parameters. Examples include:
  - `{sensor_type: 'lidar', range: 10.0, fov: 180}`
  - `{sensor_type: 'camera', resolution: [640, 480]}`

- **`arrive_mode`**:
  Chooses the method for determining if the object has arrived at its goal:
  - `'position'`: Arrival is based solely on proximity to the goal position.
  - `'state'`: Considers both position and orientation in the arrival check.

- **`description`**:
  Provides a text label or identifier for the object, which can be displayed in visualizations. It may also reference an image file for representing the object graphically.

- **`group`**:
  Assigns the object to a specific group, which can be utilized for grouping behaviors or for organizational purposes within the simulation.

- **`unobstructed`**:
  When set to `True`, the object is treated as having an unobstructed path, ignoring collisions with other objects and obstacles. This can be useful for testing or for objects that must not be impeded.

- **`plot`**:
  Contains plotting options controlling the visual representation of the object. Supported options include:
  - `'show_goal'`: Whether to display the object's goal in visualizations.
  - `'show_text'`: Display labels or descriptions with the object.
  - `'show_arrow'`: Show arrows indicating velocity or heading direction.
  - `'show_trajectory'`: Plot the path the object has taken over time.
  - Additional customization parameters like `'trail_freq'`, `'goal_color'`, `'traj_style'`, etc.

- **`state_dim`** and **`vel_dim`**:
  Specify the dimensions of the state and velocity vectors. These are typically inferred from the kinematics model but can be explicitly set if needed.

### Example Object Configurations

#### Example 1: Configuring Multiple Robots with RVO Behavior

```yaml
robot:
  - number: 5
    distribution: {name: 'random', range_low: [0, 0, -3.14], range_high: [10, 10, 3.14]}
    kinematics: {name: 'omni'}
    shape: {name: 'circle', radius: 0.2}
    behavior: {name: 'rvo', neighbor_distance: 5, max_neighbors: 10}
    goal: [5, 5, 0]
    color: 'blue'
    vel_max: [1.0, 1.0]
    sensors:
      - {sensor_type: 'lidar', range: 5.0, fov: 360}
```

#### Example 2: Configuring Static Obstacles

```yaml
obstacle:
  - shape: {name: 'rectangle', length: 2.0, width: 1.0}
    state: [3, 3, 0]
    static: True
    color: 'gray'

  - shape: {name: 'circle', radius: 1.0}
    state: [6, 6, 0]
    static: True
    unobstructed: True
    color: 'darkgreen'
```

#### Example 3: Configuring an Ackermann Steering Vehicle

```yaml
robot:
  - kinematics: {name: 'acker', wheelbase: 2.5}
    shape: {name: 'rectangle', length: 4.5, width: 2.0}
    state: [2, 2, 0]
    goal: [8, 8, 0]
    behavior: {name: 'dash'}
    vel_max: [2.0, 0.5]  # Maximum linear and steering velocities
    acce: [0.5, 0.1]
    color: 'red'
```

### Notes:

- **Multiple Objects**: When configuring multiple objects, use the `number` and `distribution` parameters to efficiently generate them. For instance, setting `number: 10` with a `distribution` of `'random'` can quickly populate the simulation with randomly placed objects.

- **Dictionary Parameters**: All dictionary-type parameters (e.g., `distribution`, `shape`, `kinematics`, `behavior`) must include a `'name'` key to specify their type. Omitting the `'name'` key will result in default values or errors.

- **Group Configurations**: By default, objects within the same group share configurations. To customize individual objects within a group, add sub-parameters using `-`. Unspecified objects will inherit the last defined configuration within the group.

- **Unobstructed Flag**: Use the `unobstructed` flag with caution. Setting it to `True` allows the object to ignore collisions, which can be useful for certain simulation scenarios but may lead to unrealistic interactions.

- **Kinematics and Velocities**: Ensure that the `velocity` and `vel_max` parameters match the kinematics model. For example, a differential drive robot (`'diff'`) should have velocities in `[v, omega]`, while an omnidirectional robot (`'omni'`) uses `[vx, vy]`.

- **Sensors**: Adding sensors to objects enhances their ability to perceive the environment. The sensor configurations require correct parameterization to function as intended.

- **Arrival Modes**: Choose the `arrive_mode` that best suits your simulation. For precise positioning, `'position'` may suffice. If orientation is critical, use `'state'` to include heading in arrival checks.

- **Plotting Options**: Customize the visualization of your simulation through the `plot` parameter. This can greatly aid in debugging and presentation by highlighting trajectories, goals, and other aspects.

By carefully configuring these parameters, you can create a rich and dynamic simulation environment tailored to your specific needs.