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
<a href="https://img.shields.io/badge/python-3.9%20%7C%203.10%20%7C%203.11%20%7C%203.12%20%7C%203.13-blue"> <img src="https://img.shields.io/badge/python-3.9%20%7C%203.10%20%7C%203.11%20%7C%203.12%20%7C%203.13-blue" alt="Python Version"></a>
</div>

**Documentation:** [https://ir-sim.readthedocs.io/en](https://ir-sim.readthedocs.io/en)

**IR-SIM** is an open-source, lightweight robot simulator based on Python, designed for robotics navigation, control, and learning. This simulator provides a simple and user-friendly framework for simulating robots, sensors, and environments, thereby lowering the barrier to developing, training, and testing AI & robotics algorithms with minimal coding and hardware requirements.

## Features

- Simulate robot platforms with diverse kinematics, sensors, and behaviors  ([support](#support)). 
- Quickly configure and customize scenarios using straightforward YAML files. No complex coding required.
- Visualize simulation outcomes using a naive visualizer matplotlib for immediate debugging.
- Support collision detection and behavior control for each object.

## Demonstrations

|                                                      Scenarios                                                        |                                                                    Description                                                                    |
| :--------------------------------------------------------------------------------------------------------------------: | :-----------------------------------------------------------------------------------------------------------------------------------------------: |
| <img src="https://github.com/user-attachments/assets/5930b088-d400-4943-8ded-853c22eae75b" alt="drawing" width="280"/> | In scenarios involving multiple circular differential robots, each robot employs Reciprocal Velocity Obstacle (RVO) behavior to avoid collisions. See [Usage - collision avoidance](https://github.com/hanruihua/ir-sim/blob/main/irsim/usage/11collision_avoidance/collision_avoidance.py) |
| <img src="https://github.com/user-attachments/assets/3257abc1-8bed-40d8-9b51-e5d90b06ee06" alt="drawing" width="280"/> |                   A car-like robot controlled via keyboard navigates a binary map using a 2D LiDAR sensor to detect obstacles.  See [Usage - grid map](https://github.com/hanruihua/ir-sim/blob/main/irsim/usage/10grid_map/grid_map.py)    |
| <img src="https://github.com/user-attachments/assets/067abec3-08a3-418d-8a1b-fe959df0b580" alt="drawing" width="280"/> |  A car-like robot controlled via keyboard navigates a grid map generated from 3D habitat spaces datasets like [HM3D](https://aihabitat.org/datasets/hm3d/), [MatterPort3D](https://niessner.github.io/Matterport/), [Gibson](http://gibsonenv.stanford.edu/database/), etc. See [Usage - grid map hm3d](https://github.com/hanruihua/ir-sim/blob/main/irsim/usage/10grid_map/grid_map_hm3d.py)|
| <img src="https://github.com/user-attachments/assets/7aa809c2-3a44-4377-a22d-728b9dbdf8bc" alt="drawing" width="280"/> |                 Each robot employing RVO behavior is equipped with a field of view (FOV) to detect other robots within this area.  See [Usage - fov](https://github.com/hanruihua/ir-sim/blob/main/irsim/usage/15fov_world/fov_world.py)               |
| <img src="https://github.com/user-attachments/assets/1cc8a4a6-2f41-4bc9-bc59-a7faff443223" alt="drawing" width="280"/> |                                  A car-like robot navigates through the randomly generated and moving obstacles. See [Usage - dynamic random obstacles](https://github.com/hanruihua/ir-sim/blob/main/irsim/usage/08random_obstacle/dynamic_random.py)                                  |


## Prerequisite

- Python: >= 3.9

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

- Or if you want to install the latest main branch version (which is more up-to-date than the PyPI version) from the source code:

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
  width: 10   # the width of the world
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


## Support

Currently, the simulator supports the following features. Further features, such as additional sensors, behaviors, and robot models, are under development.

| **Category** | **Features**                                                                                     |
| ------------ | ------------------------------------------------------------------------------------------------ |
| **Kinematics** | Differential Drive mobile Robot<br>Omni-Directional mobile Robot<br>Ackermann Steering mobile Robot |
| **Sensors**  | 2D LiDAR <br> FOV detector  |
| **Geometries** | Circle<br>Rectangle<br>Polygon <br> linestring <br> Binary Grid Map |
| **Behaviors** | dash (Move directly toward the goal)<br> rvo (Move toward the goal using Reciprocal Velocity Obstacle behavior)|


## Projects Using IR-SIM

- Academic Projects:
  - [rl-rvo-nav](https://github.com/hanruihua/rl_rvo_nav): [RAL & ICRA2023] A Reinforcement Learned based RVO behavior for multi-robot navigation.
  - [RDA_planner](https://github.com/hanruihua/RDA_planner): [RAL & IROS2023] An Accelerated Collision Free Motion Planner for Autonomous Navigation in Cluttered Environments.
  - [NeuPAN](https://github.com/hanruihua/NeuPAN): [T-RO 2025] Direct Point Robot Navigation with End-to-End Model-based Learning.

- Deep Reinforcement Learning Projects:
  - [DRL-robot-navigation-IR-SIM](https://github.com/reiniscimurs/DRL-robot-navigation-IR-SIM)

## Contributing

This project is under development. I appreciate and welcome all contributions. Just open an issue or a pull request. Here are some simple ways to start contributing:

- Enhance the website documentation, such as the API and tutorials.
- Add new sensors, behaviors, robot models, and functional interfaces.
- Add new usage examples and benchmarks.
- Report bugs and issues.

## Acknowledgement

- [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)






