<div align="center">

# Intelligent Robot Simulator (IR-SIM)

*A lightweight, YAML-driven robot simulator for navigation, control, and learning*

<a href="https://arxiv.org/pdf/2606.08729"><img src="https://img.shields.io/badge/arXiv-2606.08729-b31b1b?style=for-the-badge" alt="arXiv Paper"></a>
<a href="https://pypi.org/project/ir-sim/"><img src="https://img.shields.io/pypi/v/ir-sim?color=orange&style=for-the-badge" alt="PyPI Version"></a>
<a href="https://img.shields.io/badge/python-3.10%20%7C%203.11%20%7C%203.12%20%7C%203.13%20%7C%203.14-blue"><img src="https://img.shields.io/badge/python-3.10%20%7C%203.11%20%7C%203.12%20%7C%203.13%20%7C%203.14-blue?style=for-the-badge" alt="Python Version"></a>
<a href="https://github.com/hanruihua/ir-sim/actions/workflows/python-version-test.yml"><img src="https://img.shields.io/github/actions/workflow/status/hanruihua/ir-sim/python-version-test.yml?branch=main&style=for-the-badge&label=CI" alt="CI"></a>
<a href="https://codecov.io/gh/hanruihua/ir-sim"><img src="https://img.shields.io/codecov/c/github/hanruihua/ir-sim?style=for-the-badge&color=yellow" alt="Coverage"></a>
<a href="https://ir-sim.readthedocs.io/en/stable/"><img src="https://img.shields.io/badge/docs-online-blue?style=for-the-badge" alt="Docs"></a>
<a href="https://github.com/hanruihua/ir-sim?tab=MIT-1-ov-file"><img src="https://img.shields.io/badge/License-MIT-blue?style=for-the-badge" alt="License"></a>
<a href="https://pepy.tech/project/ir-sim"><img src="https://img.shields.io/pepy/dt/ir-sim?style=for-the-badge" alt="Downloads"></a>

</div>

## Overview

**IR-SIM** is an open-source, Python-based, lightweight robot simulator designed for navigation, control, and learning. It provides a simple, user-friendly framework with built-in collision detection for modeling robots, sensors, and environments. Ideal for academic and educational use, IR-SIM enables rapid prototyping of robotics and learning algorithms in custom scenarios with minimal coding and hardware requirements.

## Key Features

- Simulate robot platforms with diverse kinematics, sensors, and behaviors  ([support](#support)). 
- Quickly configure and customize scenarios using straightforward YAML files. No complex coding required.
- Visualize simulation outcomes using a naive visualizer matplotlib for immediate debugging.
- Support collision detection and customizable behavior policies for each object.
- Suitable for mutli-agent/robot learning ([Projects](#projects-using-ir-sim)).

## Demonstrations

<table>
<tr>
<td align="center" width="33%">
<img src="https://github.com/user-attachments/assets/5930b088-d400-4943-8ded-853c22eae75b" width="240"/><br/>
<b>Multi-Robot RVO Collision Avoidance</b><br/>
<a href="https://github.com/hanruihua/ir-sim/blob/main/usage/11collision_avoidance/collision_avoidance.py">Source</a>
</td>
<td align="center" width="33%">
<img src="https://github.com/user-attachments/assets/3257abc1-8bed-40d8-9b51-e5d90b06ee06" width="240"/><br/>
<b>Ackermann Robot with 2D LiDAR</b><br/>
<a href="https://github.com/hanruihua/ir-sim/blob/main/usage/10grid_map/grid_map.py">Source</a>
</td>
<td align="center" width="33%">
<img src="https://github.com/user-attachments/assets/0fac81e7-60c0-46b2-91f0-efe4762bb758" width="240"/><br/>
<b>HM3D / MatterPort3D Grid Map</b><br/>
<a href="https://github.com/hanruihua/ir-sim/blob/main/usage/10grid_map/grid_map_hm3d.py">Source</a>
</td>
</tr>
<tr>
<td align="center" width="33%">
<img src="https://github.com/user-attachments/assets/7aa809c2-3a44-4377-a22d-728b9dbdf8bc" width="240"/><br/>
<b>Field-of-View Detection</b><br/>
<a href="https://github.com/hanruihua/ir-sim/blob/main/usage/15fov_world/fov_world.py">Source</a>
</td>
<td align="center" width="33%">
<img src="https://github.com/user-attachments/assets/1cc8a4a6-2f41-4bc9-bc59-a7faff443223" width="240"/><br/>
<b>Dynamic Random Obstacles</b><br/>
<a href="https://github.com/hanruihua/ir-sim/blob/main/usage/08random_obstacle/dynamic_random.py">Source</a>
</td>
<td align="center" width="33%">
<img src="https://github.com/user-attachments/assets/162cf52e-070d-4588-b9b2-bf21c487fbc8" width="240"/><br/>
<b>200-Agent ORCA via <a href="https://github.com/hanruihua/pyrvo">pyrvo</a></b><br/>
<a href="https://github.com/hanruihua/ir-sim/blob/main/usage/19orca_world/orca_behavior_world.py">Source</a>
</td>
</tr>
</table>

## Installation

> **Requires Python >= 3.10**

### pip

```bash
pip install ir-sim

# Optional: keyboard control and all extras
pip install ir-sim[all]
```

### From source

```bash
git clone https://github.com/hanruihua/ir-sim.git
cd ir-sim
pip install -e .
```

### uv

```bash
git clone https://github.com/hanruihua/ir-sim.git
cd ir-sim
uv sync
```

## Quick Start

A minimal example: a differential-drive robot navigates toward a goal using the built-in `dash` behavior.

```python
import irsim

env = irsim.make('robot_world.yaml') # initialize the environment with the configuration file

for i in range(300): # run the simulation for 300 steps

    env.step()  # update the environment
    env.render() # render the environment

    if env.done(): break # check if the simulation is done

env.end() # close the environment
```

YAML Configuration: `robot_world.yaml`

```yaml
world:
  height: 10  # the height of the world
  width: 10   # the width of the world
  step_time: 0.1  # 10Hz calculate each step
  sample_time: 0.1  # 10 Hz for render and data extraction
  offset: [0, 0] # the offset of the world on x and y

robot:
  kinematics: {name: 'diff'}  # omni, omni_angular, diff, acker
  shape: {name: 'circle', radius: 0.2}  # radius
  state: [1, 1, 0]  # x, y, theta
  goal: [9, 9, 0]  # x, y, theta
  behavior: {name: 'dash'} # move toward to the goal directly
  color: 'g' # green
```

For more examples, see the [usage directory](https://github.com/hanruihua/ir-sim/tree/main/usage) and the [documentation](https://ir-sim.readthedocs.io/en).

## Support

| **Category**     | **Features**                                                                                                                                                                            |
| ---------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Kinematics**   | Differential Drive mobile Robot · Omnidirectional mobile Robot · Omnidirectional with Angular control · Ackermann Steering mobile Robot                                                 |
| **Sensors**      | 2D LiDAR · 2D FMCW LiDAR · FOV Detector                                                                                                                                                 |
| **Geometries**   | Circle · Rectangle · Polygon · LineString · Binary Grid Map · Fog of Map                                                                                                                |
| **Behaviors**    | dash (move directly toward goal) · RVO (Reciprocal Velocity Obstacle) · ORCA (Optimal Reciprocal Collision Avoidance) · SFM (Social Force Model)                                        |

## Documentation

- **English:** [https://ir-sim.readthedocs.io/en](https://ir-sim.readthedocs.io/en)
- **Chinese (中文):** [https://ir-sim.readthedocs.io/zh-cn](https://ir-sim.readthedocs.io/zh-cn)

## Projects Using IR-SIM

### Academic Publications

- **[RAL & ICRA 2023]** [rl-rvo-nav](https://github.com/hanruihua/rl_rvo_nav) -- Reinforcement learning-based RVO behavior for multi-robot navigation.
- **[RAL & IROS 2023]** [RDA_planner](https://github.com/hanruihua/RDA_planner) -- Accelerated collision-free motion planner for cluttered environments.
- **[T-RO 2025]** [NeuPAN](https://github.com/hanruihua/NeuPAN) -- Direct point robot navigation with end-to-end model-based learning.

### Community Projects

- [DRL-robot-navigation-IR-SIM](https://github.com/reiniscimurs/DRL-robot-navigation-IR-SIM) -- Deep reinforcement learning for robot navigation.
- [AutoNavRL](https://github.com/harshmahesheka/AutoNavRL) -- Autonomous navigation using reinforcement learning.
- [IRSIM-3DGS-Bridge](https://github.com/Wayneyujie/IRSIM-3DGS-Bridge) -- A closed-loop bridge from 3D Gaussian Splatting scenes to IR-SIM planning/following and back to Habitat-GS trajectory playback.

## Citation

If you find IR-SIM useful, please consider starring ⭐ this project and citing our paper:

```bibtex
@article{han2026ir,
  title={IR-SIM: A Lightweight Skill-Native Simulator for Navigation, Learning, and Benchmarking},
  author={Han, Ruihua and Wang, Shuai and Li, Chengyang and Gao, Rui and Wang, Xinyi and Liu, Zhe and Li, Guoliang and Lu, Yupu and Hao, Qi and Pan, Jia and Zhao, Hengshuang},
  journal={arXiv preprint arXiv:2606.08729},
  year={2026}
}
```

## Contributing

Contributions are welcome! Please see [CONTRIBUTING.md](https://github.com/hanruihua/ir-sim/blob/main/CONTRIBUTING.md) for guidelines.

## Acknowledgement

- [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)

## License

IR-SIM is released under the [MIT License](https://github.com/hanruihua/ir-sim?tab=MIT-1-ov-file).
