<!-- <div align="center">
<img src="doc/image/ir-sim_logos/logo1_nobg.png" width = "200" >
</div>  -->


<div align="center">

# Intelligent Robot Simulator (IR-SIM)

<a href="https://pypi.org/project/ir-sim/"><img src='https://img.shields.io/pypi/v/ir-sim?color=orange' alt='Github Release'></a>
<a href="https://github.com/hanruihua/ir-sim?tab=MIT-1-ov-file"><img src='https://img.shields.io/badge/License-MIT-blue' alt='License'></a>
<a href="https://pepy.tech/project/ir-sim"><img src="https://img.shields.io/pepy/dt/ir-sim" alt="PyPI Downloads"></a>
</div>

IR-SIM is an open-source, lightweight robot 2D simulator based on Python, specifically designed for intelligent robotics navigation and learning. Primarily intended for research and educational purposes, it is user-friendly and easily customizable.

It provides the following features:
  - A versatile and easy-to-use framework for simulating a variety of robot platforms with kinematics and sensors. 
  - Customizable configurations and parameters using yaml files.
  - Real-time visualization of simulation outcomes.
  - Ideal for developing and testing algorithms related to robot navigation, motion planning, reinforcement learning.


Robot             |  Car
:-------------------------:|:-------------------------:
![robot](https://github.com/user-attachments/assets/5930b088-d400-4943-8ded-853c22eae75b)  |  ![car](https://github.com/user-attachments/assets/3257abc1-8bed-40d8-9b51-e5d90b06ee06)

## Prerequisite

- Python: >= 3.7

## Installation

- Install this package from PyPi:

```
pip install ir-sim
```

- Or for development, you may install from source: 

```
git clone https://github.com/hanruihua/ir-sim.git    
cd ir-sim   
pip install -e .  
```
 
## Usage

### Quick Start

```python

import irsim

env = irsim.make('robot_world.yaml') # initialize the environment with the configuration file

for i in range(300): # run the simulation for 300 steps

    env.step()  # update the environment
    env.render() # render the environment

    if env.done(): break # check if the simulation is done
        
env.end() # close the environment
```

YAML Configuration: robot_world.yaml

```yaml

world:
  height: 10  # the height of the world
  width: 10   # the height of the world
  step_time: 0.1  # 10Hz calculate each step
  sample_time: 0.1  # 10 Hz for render and data extraction 
  offset: [0, 0] # the offset of the world on x and y 

robot:
  kinematics: {name: 'diff'}  # omni, diff, acker
  shape: {name: 'circle', radius: 0.2}  # radius
  state: [1, 1, 0]  # x, y, theta
  goal: [9, 9, 0]  # x, y, theta
  behavior: {name: 'dash'} # move toward to the goal directly 
  color: 'g' # green
```


### Advanced Usage

The advanced usages are listed in the [irsim/usage](https://github.com/hanruihua/ir-sim/tree/main/irsim/usage)


<!-- ## YAML Configuration Example

```yaml

world:
  height: 10  
  width: 10   
  step_time: 0.1  
  sample_time: 0.1   
  offset: [0, 0] 
  collision_mode: 'stop'  # 'stop', 'unobstructed', 'reactive'
  control_mode: 'auto'  # 'auto', 'keyboard'

robot:
  - number: 10
    distribution: {name: 'circle', radius: 4.0, center: [5, 5]}  # name: 'circle', 'random',
    kinematics: {name: 'diff'} # name: 'diff', 'omni', acker
    shape: 
      - {name: 'circle', radius: 0.2}  # name: 'circle', 'rectangle'
    behavior: {name: 'rvo', vxmax: 1.3, vymax: 1.3, accer: 1.0, factor: 0.5} # name: 
    vel_min: [-2, -2.0]
    vel_max: [2, 2.0]
    color: ['royalblue', 'red', 'green', 'orange', 'purple', 'yellow', 'cyan', 'magenta', 'lime', 'pink', 'brown'] 
    arrive_mode: position   # position, state
    goal_threshold: 0.15
    plot:
      show_trail: true
      show_goal: true
      trail_fill: True
      trail_alpha: 0.2

obstacle:
  - shape: {name: 'circle', radius: 1.0}  
    state: [5, 5, 0]  
  
  - number: 10
    distribution: {name: 'manual'}
    shape:
      - {name: 'polygon', random_shape: true, center_range: [5, 10, 40, 30], avg_radius_range: [0.5, 2], irregularity_range: [0, 1], spikeyness_range: [0, 1], num_vertices_range: [4, 5]}  

  - shape: {name: 'linestring', vertices: [[5, 5], [4, 0], [1, 6]] } 
    state: [0, 0, 0] 

``` -->

## Cases
- [rl-rvo-nav](https://github.com/hanruihua/rl_rvo_nav)(RAL & ICRA2023)
- [RDA_planner](https://github.com/hanruihua/RDA_planner)（RAL & IROS2023）


## Acknowledgement

- [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)






