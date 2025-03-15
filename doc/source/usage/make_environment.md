Make Environment
=======

## Python script and YAML configuration file

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
  width: 10   # the width of the world
  step_time: 0.1  # 10Hz calculate each step
  sample_time: 0.1  # 10 Hz for render and data extraction 
  offset: [0, 0] # the offset of the world on x and y 
  control_mode: 'auto' # 0: manual, 1: auto
  collision_mode: null # stop, react, None
  obstacle_map: null # the path of obstacle map
```

## Important Parameters Explanation 

- The `world` section specifies the properties of the world. 
- The `height` and `width` parameters specify the size of the world. 
- The `step_time` parameter specifies the time step for the simulation. 
- The `sample_time` parameter specifies the time step for rendering and data extraction. 
- The `offset` parameter specifies the offset of the world on the x and y axes. 
- The `control_mode` parameter specifies the control mode of the simulation. 
- The `collision_mode` parameter specifies the collision mode of the simulation. 
- The `obstacle_map` parameter specifies the path of the obstacle map. 

Details of the parameters can be found in the [YAML Configuration](#../yaml_config/configuration/).

:::{tip}
The default YAML configuration file is same as the name of python script. Thus, if you create a python script named `test.py`, the default YAML configuration file is `test.yaml`. And you can simply use `irsim.make()` to create the environment. Please place the YAML configuration file in the same directory as the python script.

```python
import irsim

env = irsim.make()
```
:::

