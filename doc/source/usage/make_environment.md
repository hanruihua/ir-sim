Make Environment
=======

To start the simulation, you need to create an environment. The environment is a container for all the objects in the simulation. It is also responsible for updating the state of the simulation at each time step.

Follow the steps below to create an environment:

```python
import irsim

env = irsim.make('empty_world.yaml')
```

The `make` function creates an environment from a configuration file. Support parameters can be found in [EnvBase](#irsim.env.env_base.EnvBase) class. The configuration file is a YAML file that specifies the properties of the environment. The `empty_world.yaml` file is a simple configuration file that creates an empty environment. This file is listed below:

```yaml
world:
  height: 10  # the height of the world
  width: 10   # the height of the world
  step_time: 0.1  # 10Hz calculate each step
  sample_time: 0.1  # 10 Hz for render and data extraction 
  offset: [0, 0] # the offset of the world on x and y 
  control_mode: 'auto' # 0: manual, 1: auto
  collision_mode: null # stop, react, None
  obstacle_map: null # the path of obstacle map
```


