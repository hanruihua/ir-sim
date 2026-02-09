<!-- <div align="center">
<img src="docs/image/ir-sim_logos/logo1_nobg.png" width = "200" >
</div>  -->


<div align="center">

# Intelligent Robot Simulator (IR-SIM)

<a href="https://pypi.org/project/ir-sim/"><img src='https://img.shields.io/pypi/v/ir-sim?color=orange' alt='Github Release'></a>
<a href="https://github.com/hanruihua/ir-sim?tab=MIT-1-ov-file"><img src='https://img.shields.io/badge/License-MIT-blue' alt='License'></a>
<a href="https://pepy.tech/project/ir-sim"><img src="https://img.shields.io/pepy/dt/ir-sim" alt="PyPI Downloads"></a>
<a href="https://codecov.io/gh/hanruihua/ir-sim" > <img src="https://codecov.io/gh/hanruihua/ir-sim/branch/main/graph/badge.svg?token=OSC8I5QCQ0"/> </a>
<a href="https://ir-sim.readthedocs.io/en/stable/"> <img alt="Read the Docs" src="https://img.shields.io/readthedocs/ir-sim"/> </a>
<a href="https://img.shields.io/badge/python-3.10%20%7C%203.11%20%7C%203.12%20%7C%203.13%20%7C%203.14-blue"> <img src="https://img.shields.io/badge/python-3.10%20%7C%203.11%20%7C%203.12%20%7C%203.13%20%7C%203.14-blue" alt="Python Version"></a>
</div>

**Documentation:** [https://ir-sim.readthedocs.io/en](https://ir-sim.readthedocs.io/en)

**中文文档:** [https://ir-sim.readthedocs.io/zh-cn](https://ir-sim.readthedocs.io/zh-cn)

**IR-SIM** is an open-source, Python-based, lightweight robot simulator designed for navigation, control, and reinforcement learning. It provides a simple, user-friendly framework with built-in collision detection for modeling robots, sensors, and environments. Ideal for academic and educational use, IR-SIM enables rapid prototyping of robotics and learning algorithms in custom scenarios with minimal coding and hardware requirements.

## Features

- Simulate robot platforms with diverse kinematics, sensors, and behaviors  ([support](#support)). 
- Quickly configure and customize scenarios using straightforward YAML files. No complex coding required.
- Visualize simulation outcomes using a naive visualizer matplotlib for immediate debugging.
- Support collision detection and customizable behavior policies for each object.
- Suitable for mutli-agent/robot reinforcement learning ([Reinforcement Learning Projects](#projects-using-ir-sim)).

## Demonstrations

|                                                       Scenarios                                                        |                                                                                                                                                                                       Description                                                                                                                                                                                       |
| :--------------------------------------------------------------------------------------------------------------------: | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------: |
| <img src="https://github.com/user-attachments/assets/5930b088-d400-4943-8ded-853c22eae75b" alt="drawing" width="280"/> |                                                  In scenarios involving multiple circular differential robots, each robot employs Reciprocal Velocity Obstacle (RVO) behavior to avoid collisions. See [Usage - collision avoidance](https://github.com/hanruihua/ir-sim/blob/main/usage/11collision_avoidance/collision_avoidance.py)                                                  |
| <img src="https://github.com/user-attachments/assets/3257abc1-8bed-40d8-9b51-e5d90b06ee06" alt="drawing" width="280"/> |                                                                                    A car-like robot controlled via keyboard navigates a binary map using a 2D LiDAR sensor to detect obstacles.  See [Usage - grid map](https://github.com/hanruihua/ir-sim/blob/main/usage/10grid_map/grid_map.py)                                                                                     |
| <img src="https://github.com/user-attachments/assets/0fac81e7-60c0-46b2-91f0-efe4762bb758" alt="drawing" width="280"/> | A car-like robot controlled via keyboard navigates a grid map generated from 3D habitat spaces datasets like [HM3D](https://aihabitat.org/datasets/hm3d/), [MatterPort3D](https://niessner.github.io/Matterport/), [Gibson](http://gibsonenv.stanford.edu/database/), etc. See [Usage - grid map hm3d](https://github.com/hanruihua/ir-sim/blob/main/usage/10grid_map/grid_map_hm3d.py) |
| <img src="https://github.com/user-attachments/assets/7aa809c2-3a44-4377-a22d-728b9dbdf8bc" alt="drawing" width="280"/> |                                                                                   Each robot employing RVO behavior is equipped with a field of view (FOV) to detect other robots within this area.  See [Usage - fov](https://github.com/hanruihua/ir-sim/blob/main/usage/15fov_world/fov_world.py)                                                                                    |
| <img src="https://github.com/user-attachments/assets/1cc8a4a6-2f41-4bc9-bc59-a7faff443223" alt="drawing" width="280"/> |                                                                                     A car-like robot navigates through the randomly generated and moving obstacles. See [Usage - dynamic random obstacles](https://github.com/hanruihua/ir-sim/blob/main/usage/08random_obstacle/dynamic_random.py)                                                                                     |
| <img src="https://github.com/user-attachments/assets/162cf52e-070d-4588-b9b2-bf21c487fbc8" alt="drawing" width="280"/> |                                                                                     200 agents with ORCA behavior implemented by [pyrvo](https://github.com/hanruihua/pyrvo) See [Usage - ORCA group behavior world](https://github.com/hanruihua/ir-sim/blob/main/usage/19orca_world/orca_behavior_world.py)                                                                                  |


## Prerequisite

- Python: >= 3.10

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

- If you are using `uv`

```
git clone https://github.com/hanruihua/ir-sim.git    
cd ir-sim   
uv sync
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

The advanced usages are listed in the [usage](https://github.com/hanruihua/ir-sim/tree/main/usage)


## Support

Currently, the simulator supports the following features. Further features, such as additional sensors, behaviors, and robot models, are under development.

| **Category**   | **Features**                                                                                                    |
| -------------- | --------------------------------------------------------------------------------------------------------------- |
| **Kinematics** | Differential Drive mobile Robot<br>Omni-Directional mobile Robot<br>Ackermann Steering mobile Robot             |
| **Sensors**    | 2D LiDAR <br> FOV detector                                                                                      |
| **Geometries** | Circle<br>Rectangle<br>Polygon <br> linestring <br> Binary Grid Map                                             |
| **Behaviors**  | dash (Move directly toward the goal)<br> rvo (Move toward the goal using Reciprocal Velocity Obstacle behavior) <br> orca (Move toward the goal using Optimal Reciprocal Collision Avoidance group behavior) |


## Projects Using IR-SIM

- Academic Projects:
  - [rl-rvo-nav](https://github.com/hanruihua/rl_rvo_nav): [RAL & ICRA2023] A Reinforcement Learned based RVO behavior for multi-robot navigation.
  - [RDA_planner](https://github.com/hanruihua/RDA_planner): [RAL & IROS2023] An Accelerated Collision Free Motion Planner for Cluttered Environments.
  - [NeuPAN](https://github.com/hanruihua/NeuPAN): [T-RO 2025] Direct Point Robot Navigation with End-to-End Model-based Learning.

- Deep Reinforcement Learning Projects:
  - [DRL-robot-navigation-IR-SIM](https://github.com/reiniscimurs/DRL-robot-navigation-IR-SIM)
  - [AutoNavRL](https://github.com/harshmahesheka/AutoNavRL)

## Contributing

This project is under development. I appreciate and welcome all contributions. Just open an issue or a pull request. Please refer to the [CONTRIBUTING.md](https://github.com/hanruihua/ir-sim/blob/main/CONTRIBUTING.md) for more details.

## Acknowledgement

- [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)






