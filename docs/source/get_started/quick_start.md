# Quick Start

To quickly start the simulation, you can use the following code snippet to run a simulation for a robot in a world.

1. Create a Python file and copy the following code snippet to run the simulation.

```python
import irsim

env = irsim.make('robot_world.yaml') # initialize the environment with the configuration file

for i in range(300): # run the simulation for 300 steps

    env.step()  # update the environment
    env.render() # render the environment

    if env.done(): 
        break # check if the simulation is done
        
env.end() # close the environment
```

2. Create a configuration YAML file *robot_world.yaml* and copy the following configuration to the file.

All the configurations are set in the YAML file. You can change the configurations in the YAML file to customize the simulation. The following is an example of the configuration file *robot_world.yaml*.

```yaml
world:
  height: 10  # the height of the world
  width: 10   # the width of the world
  step_time: 0.1  # 10Hz to calculate each step
  sample_time: 0.1  # 10 Hz for render and data extraction 
  offset: [0, 0] # the offset of the world on x and y 

robot:
  kinematics: {name: 'diff'}  # kinematics of the robot, current name should be one of omni, omni_angular, diff, acker. If not set, this object will be static
  shape: {name: 'circle', radius: 0.2}  # radius for circle shape
  state: [1, 1, 0]  # x, y, theta, 2d position and orientation
  goal: [9, 9, 0]  # x, y, theta, 2d position and orientation
  behavior: {name: 'dash'} # move toward the goal directly 
  color: 'g' # green
```

## Next steps

- [Make Environment](../usage/make_environment.md) — the simulation loop, status control, and dynamic objects in depth.
- [Configure robots and obstacles](../usage/configure_robots_obstacles.md) — kinematics, shapes, and multi-object scenes.
- [YAML Configuration](../yaml_config/configuration.md) — the full configuration reference with an interactive schema explorer.