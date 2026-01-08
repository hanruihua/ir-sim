Make Environment
==================

## Python script and YAML configuration file

To start the simulation, you need to create an environment. The environment is a container for all the objects in the simulation. It is also responsible for updating the state of the simulation at each time step.

::::{tab-set}
:::{tab-item} Python Script

Create your environment with a simple Python script:

```python
import irsim

env = irsim.make('empty_world.yaml')
```
The `make` function creates an environment from a configuration file. Supported parameters include:

- **`world_name`** (str, optional): Path to the world YAML configuration file
- **`projection`** (str, optional): Projection type ("3d" for 3D environment, None for 2D)
- **`display`** (bool): Whether to display the environment visualization (default: True)
- **`save_ani`** (bool): Whether to save the simulation as an animation (default: False)
- **`log_level`** (str): Logging level for the environment (default: "INFO")
- **`seed`** (int, optional): Seed for IR-SIM's project RNG. If provided,
  random elements produced by IR-SIM become reproducible. If omitted/``None``,
  a new unseeded generator is used (non-reproducible). Custom extensions using
  ``np.random`` or Python ``random`` should either switch to IR-SIM's RNG or be
  seeded separately.

For more details, see the [EnvBase](#irsim.env.env_base.EnvBase) class documentation.
:::

:::{tab-item} YAML Configuration

Define your world properties in a YAML file (`empty_world.yaml`):

```yaml
world:
  height: 10  # the height of the world (meters)
  width: 10   # the width of the world (meters)
  step_time: 0.1  # simulation time step (seconds) - 10Hz
  sample_time: 0.1  # rendering frequency (seconds) - 10Hz
  offset: [0, 0] # the offset of the world origin [x, y]
  control_mode: 'auto' # control mode: 'auto', 'keyboard'
  collision_mode: 'stop' # collision behavior: 'stop', 'unobstructed', 'unobstructed_obstacles'
  obstacle_map: null # path to obstacle map file (optional)
```

The configuration file is a YAML file that specifies the properties of the environment. The `empty_world.yaml` file is a simple configuration file that creates an empty environment.
:::
::::

## Important Parameters Explanation 

### World Configuration

- **`world`**: Main section that defines the simulation environment properties
- **`height`** and **`width`**: Specify the size of the simulation world in meters
- **`step_time`**: Controls simulation accuracy and speed (smaller = more accurate, slower)
- **`sample_time`**: Controls rendering frequency (larger = faster simulation, less smooth visualization)
- **`offset`**: Shifts the world coordinate system origin [x, y] in meters
- **`control_mode`**: Determines how the simulation is controlled
  - `'auto'`: Automatic simulation execution
  - `'keyboard'`: Manual keyboard control
- **`collision_mode`**: Defines collision detection behavior
  - `'stop'`: Stop simulation when collision occurs (default)
  - `'unobstructed'`: Ignore all collisions
  - `'unobstructed_obstacles'`: Ignore only obstacle collisions
- **`obstacle_map`**: Optional path to a pre-defined obstacle map file

### Performance Considerations

- **Smaller `step_time`**: More accurate physics but slower simulation
- **Larger `sample_time`**: Faster simulation but less smooth visualization  
- **World size**: Larger worlds require more computational resources

```{tip}
You can use `sample_time` to control the rendering frequency and accelerate the simulation speed. The default value of `sample_time` is the same as `step_time`.
```

For more detailed parameter information, see the [YAML Configuration](../yaml_config/index.rst) reference.

:::{tip}
**Automatic Configuration Detection**: The default YAML configuration file has the same name as your Python script. For example, if you create a script named `test.py`, IR-SIM automatically looks for `test.yaml` in the same directory.

```python
import irsim

# Automatically uses 'test.yaml' if this file is 'test.py'
env = irsim.make()
```

This feature simplifies development by eliminating the need to specify configuration files explicitly.
:::

## Basic Simulation Loop

After creating an environment, you'll typically run a simulation loop:

```python
import irsim

env = irsim.make('config.yaml')

# Main simulation loop
for i in range(1000):
    env.step()        # Update simulation state
    env.render(0.05)  # Render with 0.05 second interval (20Hz)
    
    if env.done():    # Check if simulation should end
        break

env.end()  # Clean up resources
```

### Core Methods Explained

- **`env.step()`**: Advances the simulation by one time step
- **`env.render(interval)`**: Updates the visualization with specified time interval between frames
- **`env.done()`**: Returns `True` if simulation completion conditions are met
- **`env.end()`**: Properly closes the environment and releases resources

::::{tip}
Update order

The environment advances all objects first, then updates all sensors. This two-phase update ensures sensors read the latest world state consistently. If you step objects manually, either pass `sensor_step=True` to `ObjectBase.step(...)` or call `obj.sensor_step()` after updating states.
::::

## Environment Control and Status

### Status Management

```python
import irsim

env = irsim.make('config.yaml')

# Check current status
print(f"Current status: {env.status}")
print(f"Current time: {env.time}")

# Control simulation state
env.pause()    # Pause the simulation
env.resume()   # Resume the simulation

# Simulation loop with status checking
for i in range(50):
    env.step()
    env.render(0.05)

    if i < 10:
        env.pause()
    elif i > 20 and i < 30:
        env.resume()
    
    if env.status == "Pause":
        print("Environment is paused")
    
    if env.done():
        print("Simulation completed successfully")

env.end()
```

### Available Status Values

- **`"Running"`**: The environment is running in auto control mode
- **`"Running (keyboard)"`**: The environment is running in keyboard control mode
- **`"Pause"`**: Simulation is paused
- **`"Done"`**: Simulation has completed
- **`"Arrived"`**: All robots have arrived at their goals
- **`"Collision"`**: A collision has occurred
- **`"Pause (Debugging)"`**: Debugging mode (when `F5` key is pressed)
- **`"Reset"`**: The environment is reset
- **`"Reload"`**: The environment is reloaded
- **`"Save Figure"`**: The figure is saved
- **`"Quit"`**: The environment is quit


## Configure Environment Title

By default, the simulation time and status are shown in the environment title. You can customize this behavior by setting the `show_title` and customizing the title by `env.set_title()`.

::::{tab-set}
:::{tab-item} Custom Title Configuration

```python
import irsim

env = irsim.make('config.yaml')

# Set custom title
env.set_title("Multi-Robot Navigation Simulation")

# Update title dynamically
for i in range(100):
    env.step()
    
    # Update title every 10 steps
    if i % 10 == 0:
        env.set_title(f"Simulation Step: {i}")
    
    env.render(0.05)

env.end()
```
:::

:::{tab-item} Enable/Disable Title Display

```yaml
world:
  height: 20
  width: 20
  control_mode: 'auto'
  plot:
    show_title: true    # Show title with time and status
    show_axis: true     # Optional: show axis labels
```
:::
::::


## Multiple Environments

IR-SIM supports creating and running multiple environments simultaneously. Each environment maintains its own isolated state, including world parameters, objects, and simulation time. This is useful for:

- **Parallel simulations**: Running multiple scenarios simultaneously
- **Comparison studies**: Comparing different algorithms or configurations
- **Training**: Running multiple instances for reinforcement learning

### Creating Multiple Environments

```python
import irsim

# Create two separate environments
env1 = irsim.make('scenario_a.yaml')
env2 = irsim.make('scenario_b.yaml')

# Each environment has its own state
print(f"Env1 robots: {env1.robot_number}")
print(f"Env2 robots: {env2.robot_number}")
```

### Isolated Parameters

Each environment has completely separate parameter instances:

```python
import irsim

env1 = irsim.make('world1.yaml')
env2 = irsim.make('world2.yaml')

# World parameters are isolated
env1.world_param.control_mode = 'keyboard'
print(f"Env1 control mode: {env1.world_param.control_mode}")  # keyboard
print(f"Env2 control mode: {env2.world_param.control_mode}")  # auto (unchanged)

# Simulation time is independent
for _ in range(10):
    env1.step()

for _ in range(5):
    env2.step()

print(f"Env1 time: {env1.time}")  # 1.0 (10 steps * 0.1)
print(f"Env2 time: {env2.time}")  # 0.5 (5 steps * 0.1)
```

### Running Multiple Environments

::::{tab-set}

:::{tab-item} Sequential Execution

```python
import irsim

env1 = irsim.make('scenario_a.yaml', display=True)
env2 = irsim.make('scenario_b.yaml', display=True)

# Run both environments in the same loop
for i in range(500):
    # Step both environments
    env1.step()
    env2.step()

    # Render both (creates two windows)
    env1.render(0.01)
    env2.render(0.01)

    # Check completion independently
    if env1.done() and env2.done():
        break

env1.end()
env2.end()
```
:::

:::{tab-item} Headless Parallel (for Training)

```python
import irsim

# Create multiple headless environments for training
num_envs = 4
envs = [
    irsim.make(f'training_world.yaml', display=False, seed=i)
    for i in range(num_envs)
]

# Run training loop
for episode in range(100):
    # Reset all environments
    for env in envs:
        env.reset()

    # Collect experiences from all environments
    for step in range(1000):
        actions = [get_action(env) for env in envs]  # Your policy

        for env, action in zip(envs, actions):
            env.step(action)

        # Check if any environment is done
        if all(env.done() for env in envs):
            break

# Clean up
for env in envs:
    env.end()
```
:::

::::

### Parameter Isolation Details

Each environment instance creates and binds its own parameter objects:

**`world_param`** - Simulation state and settings:

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| `time` | `float` | `0.0` | Current simulation time |
| `control_mode` | `str` | `"auto"` | Control mode (`"auto"` or `"keyboard"`) |
| `collision_mode` | `str` | `"stop"` | Collision handling mode |
| `step_time` | `float` | `0.1` | Time step duration (seconds) |
| `count` | `int` | `0` | Simulation step counter |

**`env_param`** - Environment objects and utilities:

| Attribute | Type | Description |
|-----------|------|-------------|
| `objects` | `list[ObjectBase]` | List of all objects in the environment |
| `logger` | `EnvLogger` | Logger instance for this environment |
| `GeometryTree` | `STRtree` | Spatial index for collision detection |
| `platform_name` | `str` | Operating system name |

**`path_param`** - File path management:

| Attribute | Type | Description |
|-----------|------|-------------|
| `root_path` | `str` | Path to irsim package directory |
| `ani_buffer_path` | `str` | Path for animation frame buffer |
| `ani_path` | `str` | Path for saved animations |
| `fig_path` | `str` | Path for saved figures |

Objects within each environment reference their own environment's parameters:

:::{note}
When creating multiple environments with `display=True`, each environment opens its own visualization window. For training or batch simulations, use `display=False` to disable rendering and improve performance.
:::

