Configure dynamic random environment
====================================

The dynamic, random, and clutter environment is useful to test the navigation and collision avoidance algorithms. In this environment, the distribution, goal position, and geometry of the obstacles are random at each episode. 


## Random Obstacles Configuration Parameters

Python script:

```python
import irsim

env = irsim.make()   

for i in range(1000):

    env.step()
    env.render(0.05)

    if env.done():
        break

env.end()
```

YAML File:

```yaml
world:
  height: 50  # the height of the world
  width: 50   # the width of the world
  control_mode: 'keyboard'  

robot:
  - kinematics: {name: 'acker'} 
    shape: {name: 'rectangle', length: 4.6, width: 1.6, wheelbase: 3}
    state: [5, 5, 0, 0]
    goal: [40, 40, 0]
    vel_max: [4, 1]

    sensors: 
      - type: 'lidar2d'
        range_min: 0
        range_max: 20
        angle_range: 3.14
        number: 100
    
    plot:
      show_trajectory: True

obstacle:

  - number: 20
    kinematics: {name: 'omni'}
    distribution: {name: 'random', range_low: [10, 10, -3.14], range_high: [40, 40, 3.14]}
    behavior: {name: 'rvo', wander: True, range_low: [10, 10, -3.14], range_high: [40, 40, 3.14], vxmax: 2, vymax: 2, factor: 3.0}
    vel_max: [4, 4]
    vel_min: [-4, -4]
    shape:
      - {name: 'circle', radius: 1.0, random_shape: True}  
      - {name: 'polygon', random_shape: true, avg_radius_range: [0.5, 2.0], irregularity_range: [0, 0.4], spikeyness_range: [0, 0.4], num_vertices_range: [4, 6]}
```

The demonstration is shown in the following figure:

```{image} gif/random_obstacles.gif
:alt: random_obstacles
:width: 400px
:align: center
```

## Important Parameters Explanation 

The parameters to generate random obstacles with various shapes are the settings of `behavior`, `distribution`, and `shape` in the `obstacle` section.

- For the `rvo` behavior, set the `wander` to `True` to enable the random movement of the obstacles when they reach the goal position. And the `rvo` behavior is used to avoid the collision among obstacles. `range_low` and `range_high` are the lower and upper bounds of the random distribution of the goal position of the obstacles. 

- The `distribution` parameter with name `random` is used to set the random distribution of the obstacles in a certain area. The `range_low` and `range_high` are the lower and upper bounds of the random distribution of the initial position of the obstacles.

- The `shape` parameter is used to set the random shape of the obstacles by setting the `random_shape` to `True`. 
  - For circular obstacles, the `radius` will be randomly generated within `radius_range`. 
  - For polygon obstacles, `avg_radius_range`, `irregularity_range`, `num_vertices_range`, and `spikeyness_range` define which type of polygon will be generated. See [random_generate_polygon](#irsim.lib.algorithm.generation.random_generate_polygon) for details.

The details of the parameters are listed in the [YAML Configuration](../yaml_config/configuration/)

:::{tip}
To generate a convex polygon, you can set `spikeyness_range` to [0, 0] or set `is_convex` to True.
:::