Configure 2D LiDAR for the robot
=================================

To simulate a robot with a 2D LiDAR sensor, you need to define and configure the LiDAR sensor parameters. 

## LiDAR Configuration Parameters

The YAML configuration file and Python Script below shows an example of a robot with a 2D LiDAR sensor:

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

YAML file (same name as the python script):

```yaml
world:
  height: 10  
  width: 10   

robot:
  - kinematics: {name: 'diff'}  # omni, diff, acker
    shape: {name: 'circle', radius: 0.2}  # radius
    goal: [9, 9, 0]

    sensors:
      - name: 'lidar2d'
        range_min: 0
        range_max: 5
        angle_range: 3.14 
        number: 200
        noise: False
        alpha: 0.3
      
obstacle:
  - shape: {name: 'circle', radius: 1.0}  # radius
    state: [5, 5, 0]  
  
  - shape: {name: 'rectangle', length: 1.5, width: 1.2}  # length, width
    state: [6, 5, 1] 
  
  - shape: {name: 'linestring',vertices: [[10, 5], [4, 0], [6, 7]]}  # vertices
    state: [0, 0, 0] 
```



The demonstration shows below:

```{image} gif/lidar2d.gif
:alt: Select Parameters
:width: 400px
:align: center
```

### Important Parameters Explained

To configure the 2D LiDAR sensor, the sensor name of `lidar2d` should be defined in the `sensors` section of the robot. Key parameters of the LiDAR sensor are explained below:

- **range_min**: The minimum range of the laser beam.
- **range_max**: The maximum range of the laser beam.
- **angle_range**: The angle range of the laser beam.
- **number**: The number of beams.
- **alpha**: The transparency of the laser beam.

A full list of parameters can be found in the [YAML Configuration](#../yaml_config/configuration/).


## Advanced Configuration with noise

To add noise to the LiDAR sensor, you can set the `noise` parameter to `True`. The YAML configuration file:

```yaml
world:
  height: 10  
  width: 10   

robot:
  - kinematics: {name: 'diff'}  # omni, diff, acker
    shape: {name: 'circle', radius: 0.2}  # radius
    goal: [9, 9, 0]

    sensors:
      - type: 'lidar2d'
        range_min: 0
        range_max: 5
        angle_range: 3.14 #  4.7123
        number: 200
        noise: True
        std: 0.1
        angle_std: 0.2
        offset: [0, 0, 0]
        alpha: 0.3
      
obstacle:
  - shape: {name: 'circle', radius: 1.0}  # radius
    state: [5, 5, 0]  
  
  - shape: {name: 'rectangle', length: 1.5, width: 1.2}  # length, width
    state: [6, 5, 1] 
  
  - shape: {name: 'linestring',vertices: [[10, 5], [4, 0], [6, 7]]}  # vertices
    state: [0, 0, 0] 

```

Gaussian noise is added to the LiDAR sensor with the `std` and `angle_std` parameters. The `std` parameter is the standard deviation of the range noise, and the `angle_std` parameter is the standard deviation of the angle noise. 
