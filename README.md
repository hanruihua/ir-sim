<!-- <div align="center">
<img src="doc/image/ir-sim_logos/logo1_nobg.png" width = "200" >
</div>  -->


<div align="center">

# Intelligent Robot Simulator (IR-SIM)

<a href="https://pypi.org/project/ir-sim/"><img src='https://img.shields.io/pypi/v/ir-sim?color=orange' alt='Github Release'></a>
<a href="https://github.com/hanruihua/ir-sim?tab=MIT-1-ov-file"><img src='https://img.shields.io/badge/License-MIT-blue' alt='License'></a>
<a href="https://pepy.tech/project/ir-sim"><img src="https://img.shields.io/pepy/dt/ir-sim" alt="PyPI Downloads"></a>
<a href="https://codecov.io/gh/hanruihua/ir-sim" > <img src="https://codecov.io/gh/hanruihua/ir-sim/branch/dev/graph/badge.svg?token=OSC8I5QCQ0"/> </a>
<a href="https://ir-sim.readthedocs.io/en/latest/"> <img alt="Read the Docs" src="https://img.shields.io/readthedocs/ir-sim"/> </a>

</div>

**Documentation:** [https://ir-sim.readthedocs.io/en](https://ir-sim.readthedocs.io/en/latest/)

IR-SIM is an open-source, lightweight robot simulator based on Python, specifically designed for intelligent robotics navigation and learning. Primarily intended for research and educational purposes, it is user-friendly and easily customizable.

It provides the following features:
  - A versatile and easy-to-use framework for simulating a variety of robot platforms with kinematics and sensors. 
  - Customizable configurations and parameters using yaml files.
  - Real-time visualization of simulation outcomes.
  - Ideal for developing and testing algorithms related to robot navigation, motion planning, reinforcement learning.

|                                           Robot                                           |                                           Car                                           |
| :---------------------------------------------------------------------------------------: | :-------------------------------------------------------------------------------------: |
| ![robot](https://github.com/user-attachments/assets/5930b088-d400-4943-8ded-853c22eae75b) | ![car](https://github.com/user-attachments/assets/3257abc1-8bed-40d8-9b51-e5d90b06ee06) |

## Prerequisite

- Python: >= 3.7

## Installation

- Install this package from PyPi:

```
pip install ir-sim
```

This does not include dependencies for all features of the simulator. To install additional optional dependencies, use the following pip commands:

```
# install dependencies for keyboard control
pip install ir-sim[keyboard]

# install all optional dependencies
pip install ir-sim[all]  
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

## Academic Cases
- [rl-rvo-nav](https://github.com/hanruihua/rl_rvo_nav) (RAL & ICRA2023)
- [RDA_planner](https://github.com/hanruihua/RDA_planner)（RAL & IROS2023）


## Acknowledgement

- [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)






