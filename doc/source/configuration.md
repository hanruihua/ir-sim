Configuration
==================

The configuration file is a YAML file that contains the configuration of the world and the robot. The configuration file is used to initialize the environment. 

The configuration file is divided into three sections: `world`, `robot`, and `obstacle`. The `world` section contains the configuration of the world, such as the size of the world and the goal position. The `robot` section contains the configuration of the robot, such as the initial position, kinematics, and goal position. The `obstacles` section contains the configuration of the obstacles in the world. Following is a simple example of the configuration file:

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
  - shape: {name: 'circle', radius: 1.0}  # radius
    state: [5, 5, 0]  
  
  - shape: {name: 'rectangle', length: 1.5, width: 1.2}  # radius
    state: [6, 5, 1] 

  - shape: {name: 'linestring', vertices: [[5, 5], [4, 0], [1, 6]] }  # vertices
    state: [0, 0, 0] 
    unobstructed: True
```

Note that, you can have multiple robots and obstacles in the configuration file by adding multiple entries in the `robot` and `obstacle` sections with '-'. 

## World Configuration

The `world` section contains the configuration of the simulation environment. The following table details the configuration parameters for the world:

| Parameter        | Description                                                                 | Type               | Default       |
|------------------|-----------------------------------------------------------------------------|--------------------|---------------|
| `name`           | Name of the world                                                           | `str`              | `"world"`     |
| `height`         | Height of the world                                                         | `float`            | `10`          |
| `width`          | Width of the world                                                          | `float`            | `10`          |
| `step_time`      | Time interval between simulation steps (in seconds)                         | `float`            | `0.1`         |
| `sample_time`    | Time interval between samples for rendering and data extraction (in seconds) | `float`            | `0.1`         |
| `offset`         | Offset for the world's position in `[x, y]` coordinates                    | `list` of `float`  | `[0, 0]`      |
| `control_mode`   | Control mode of the simulation (`"auto"` or `"keyboard"`)                  | `str`              | `"auto"`      |
| `collision_mode` | Collision handling mode (`"stop"`, `"reactive"`, `"unobstructed"`)         | `str`              | `"stop"`      |
| `obstacle_map`   | Path to the image file representing the obstacle map                        | `str` (file path)  | `None`        |
| `mdownsample`    | Downsampling factor for the obstacle map to reduce resolution                | `int`              | `1`           |

### Description of Parameters

- **`name`**:  
  Defines the name of the world used in the simulation. This can be useful for identifying different simulation environments.

- **`height`**:  
  Specifies the vertical size of the world in units of measurement defined by the simulation (e.g., meters).

- **`width`**:  
  Specifies the horizontal size of the world in units of measurement defined by the simulation.

- **`step_time`**:  
  Determines the time interval between each simulation step. A smaller `step_time` results in a higher simulation frequency (e.g., `0.1` seconds corresponds to 10 Hz).

- **`sample_time`**:  
  Defines the time interval for rendering the simulation and extracting data. This controls how frequently visual updates and data recordings occur.

- **`offset`**:  
  Sets the initial positional offset of the world on the X and Y axes. This is useful for positioning the world within a larger coordinate system or for relative placement.

- **`control_mode`**:  
  Configures how the simulation is controlled:
  - `"auto"`: Automatic control by the simulation logic.
  - `"keyboard"`: Manual control via keyboard inputs.

- **`collision_mode`**:  
  Defines how collisions between objects are handled in the simulation:
  - `"stop"`: Stops the movement of objects upon collision.
  - `"reactive"`: Objects react to collisions based on predefined behaviors.
  - `"unobstructed"`: Allows objects to pass through each other without any collision response.

- **`obstacle_map`**:  
  Specifies the file path to an image that serves as the obstacle map. This image is used to generate the grid map that defines the positions of obstacles within the world.

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

- **`mdownsample`**: Adjust the downsampling factor based on the desired resolution and performance requirements. A value of `1` retains the original resolution, while higher values reduce it.

- **Customization**: Modify the parameter values as needed to suit the specific requirements of your simulation environment. For instance, changing `control_mode` to `'keyboard'` allows manual control, which can be useful for interactive simulations.

By completing the **World Configuration** section as shown above, you ensure that all relevant parameters from `world.py` are accurately represented and configurable via the YAML configuration file.

## Robot Configuration



## Obstacle Configuration






