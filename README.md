<!-- <div align="center">
<img src="doc/image/IR_SIM_logos/logo1_nobg.png" width = "200" >
</div>  -->


<div align="center">

# Intelligent Robot Simulator (IR-SIM)

<a href="https://img.shields.io/badge/release-v2.1.0-brightgreen?link=https%3A%2F%2Fgithub.com%2Fhanruihua%2Fir_sim%2Freleases%2F
)](https://github.com/hanruihua/ir_sim/releases/"><img src='https://img.shields.io/github/v/release/hanruihua/ir_sim?color=brightgreen' alt='Github Release'></a>
<a href="https://github.com/hanruihua/ir_sim?tab=MIT-1-ov-file"><img src='https://img.shields.io/badge/License-MIT-blue' alt='License'></a>
<a href="https://pypistats.org/packages/ir-sim"><img src='https://img.shields.io/pypi/dm/ir_sim' alt='Download'></a>

</div>

IR-SIM is an open-source, lightweight robot 2D simulator based on Python, specifically designed for intelligent robotics navigation and learning. Primarily intended for research and educational purposes, it is user-friendly and easily customizable.

It provides the following features:
  - A versatile and easy-to-use framework for simulating a variety of robot platforms with kinematics and sensors. 
  - Customizable configurations and parameters using yaml files.
  - Real-time visualization of simulation outcomes.
  - Ideal for developing and testing algorithms related to robot navigation, motion planning, reinforcement learning.


Robot             |  Car
:-------------------------:|:-------------------------:
![robot](doc/animations/rvo.gif)  |  ![car](doc/animations/car.gif)


## Prerequisite

- Python: >= 3.9

## Installation

- Install this package from PyPi:

```
pip install ir_sim
```

- Or for development, you may build from source: 

```
git clone https://github.com/hanruihua/ir_sim.git    
cd ir_sim   
pip install -e .  
```

## Usage

The usage guidelines are listed in the [ir_sim/usage](https://github.com/hanruihua/ir_sim/tree/main/ir_sim/usage)

## Cases
- [rl-rvo-nav](https://github.com/hanruihua/rl_rvo_nav)(RAL & ICRA2023)
- [RDA_planner](https://github.com/hanruihua/RDA_planner)（RAL & IROS2023）


<!-- ## Contact: 
hanrh@connect.hku.hk -->





