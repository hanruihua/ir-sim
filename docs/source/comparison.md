# When to use IR-SIM

IR-SIM occupies a specific niche: a **lightweight, pure-Python, YAML-driven 2D simulator** for navigation, control, and reinforcement learning. This page explains where it shines and where another tool is the better choice, so you can pick the right simulator for your problem.

## A good fit when you want to

- **Prototype and benchmark** navigation, control, or RL algorithms in 2D, quickly.
- **Describe scenes declaratively** in YAML — iterate fast, version-control scenarios, and share them.
- Run **multi-agent** experiments with built-in collision-avoidance behaviors (RVO, ORCA, SFM).
- Stay **pure-Python and pip-installable**, with minimal dependencies and no GPU.

## Reach for another tool when you need

- **High-fidelity 3D physics**, contact dynamics, or manipulation → Gazebo, MuJoCo, or NVIDIA Isaac Sim.
- **Photorealistic sensors** or autonomous-driving scenes → CARLA.
- A tightly **ROS-integrated** robot stack for sim-to-real → Gazebo with ROS.

## How it compares

| Tool | Focus | Language | Fidelity | Best for |
| --- | --- | --- | --- | --- |
| **IR-SIM** | 2D navigation / control / RL | Python | lightweight, kinematic | fast prototyping, YAML scenes, `pip install` |
| Gazebo | general 3D robotics | C++ / ROS | high (3D physics) | sim-to-real, full robot stacks |
| CARLA | autonomous driving | C++ / Python | photorealistic 3D | driving research, rich sensors |
| Stage | 2D multi-robot | C++ / ROS | lightweight 2D | classic 2D multi-robot with ROS |
| Gymnasium / PettingZoo | RL environment API | Python | — (interface) | the standard RL env interface |
| highway-env | 2D driving RL | Python | lightweight 2D | driving-specific RL benchmarks |

A couple of clarifications:

- **Gymnasium and PettingZoo are interfaces, not simulators.** They standardise how an agent talks to an environment; IR-SIM can sit *behind* such an interface to provide the dynamics.
- IR-SIM deliberately trades physical fidelity for **minimal setup and speed of iteration** — it models kinematics and collisions, not contact forces or photorealism.

## In short

If you want the fastest path from an idea to a running navigation, control, or multi-agent RL experiment — and you don't need 3D physics or photorealism — IR-SIM is built for exactly that. When fidelity matters more than iteration speed, pair it with (or graduate to) one of the heavier simulators above.

Ready to try it? Head to {doc}`Getting Started <get_started/index>`.
