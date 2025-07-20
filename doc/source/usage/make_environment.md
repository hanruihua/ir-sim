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
The `make` function creates an environment from a configuration file. Supported parameters can be found in [EnvBase](#irsim.env.env_base.EnvBase) class.
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
  collision_mode: 'stop' # collision behavior: 'stop',  , 'unobstructed', 'unobstructed_obstacles'
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
    env.render(0.05)  # Render with 20Hz
    
    if env.done():    # Check if simulation should end
        break

env.end()  # Clean up resources
```

### Core Methods Explained

- **`env.step()`**: Advances the simulation by one time step
- **`env.render(frame_rate)`**: Updates the visualization with specified frame rate
- **`env.done()`**: Returns `True` if simulation completion conditions are met
- **`env.end()`**: Properly closes the environment and releases resources

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

- **`"Running"`**: Simulation is actively running
- **`"Pause"`**: Simulation is paused
- **`"Done"`**: Simulation has completed
- **`"Arrived"`**: All robots have arrived at their goals
- **`"Collision"`**: A collision has occurred

## Configure Environment Title

By default, the simulation time and status are shown in the environment title. You can customize this behavior by setting the `show_title` and customizing the title by `env.set_title()`.

### Enable/Disable Title Display

```yaml
world:
  height: 20
  width: 20
  control_mode: 'auto'
  plot:
    show_title: true    # Show title with time and status
    show_axis: true     # Optional: show axis labels
```

### Custom Title Configuration

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