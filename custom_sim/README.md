# Custom LiDAR Robot Simulator & RL Training Stack

A lightweight, custom-built 2D robot simulator designed for Reinforcement Learning (RL) research. It features a Gymnasium-compliant environment, LiDAR interactions with geometric obstacles (circles, rectangles), and a complete PPO training pipeline using Stable-Baselines3.

## 🚀 Features

*   **Custom Physics Engine**: Lightweight 2D kinematics for Differential Drive and Ackermann robots.
*   **LiDAR Simulation**: Ray-casting sensor with support for Circular and Rotated Rectangular obstacles.
*   **Gymnasium Interface**: Standard `gym.Env` wrapper (`LidarNavigationEnv`) for easy integration with RL libraries.
*   **RL Pipeline**: Full training and testing scripts using PPO (Proximal Policy Optimization).
*   **Visualization**: Real-time 2D rendering of the robot, Lidar rays, and path planning.
*   **Navigation Stack**: Includes A* (Global Planner) and DWA (Local Planner) for baseline comparison.

## 🛠️ Installation

Ensure you have Python 3.8+ installed.

1.  **Install Dependencies**:
    ```bash
    uv pip install -r requirements.txt
    ```

    *Key libraries: `gymnasium`, `stable-baselines3`, `torch`, `matplotlib`, `numpy`.*

## 🏃‍♂️ Usage

### 1. Manual / Algorithmic Simulation
Run the classic navigation stack (A* + DWA) to verify the physics and map logic.

```bash
uv run python main.py
```
*   **Controls**: The robot navigates autonomously to the goal defined in `robot_world.yaml`.

### 2. Reinforcement Learning (RL)

#### Train a New Agent
Train a PPO agent from scratch. The model learns to navigate to the goal using only LiDAR and relative goal position.

```bash
uv run python train_agent.py --mode train
```
*   **Output**: Logs are saved to `logs/`, models to `models/`.
*   **Note**: Training 500k steps takes approx. 10-15 minutes on a standard laptop.

#### Test the Trained Agent
Visualize the trained policy in action.

```bash
uv run python train_agent.py --mode test --model-path models/final_model
```
*   **Headless Mode** (Metrics only):
    ```bash
    uv run python train_agent.py --mode test --model-path models/final_model --no-render
    ```

#### Visualize Training Progress
Launch TensorBoard to see reward curves.

```bash
uv run tensorboard --logdir logs/
```

## 📂 Project Structure

```text
├── main.py             # Entry point for Manual/Algorithmic simulation
├── train_agent.py      # Entry point for RL Training & Testing
├── robot_world.yaml    # Configuration file (World size, Obstacles, Robot goal)
│
├── rl_env.py           # Gymnasium Environment Wrapper (The bridge between Sim and RL)
├── world.py            # World state manager (Robots, Obstacles, Time)
├── entities.py         # Entity classes (Robot, Obstacle)
├── sensor.py           # LiDAR sensor logic (Ray casting math)
├── navigation.py       # Classic planners (A*, DWA)
├── mapping.py          # Occupancy Grid mapping
│
└── models/             # Directory where trained models are saved
    └── final_model.zip # The provided pre-trained agent (100% Success Rate)
```

## 🧠 Approach

1.  **Simulation**: A custom `World` class steps through physics `dt=0.1s`.
2.  **Perception**: The `Lidar2D` class casts 360 rays. It computes intersections with Circles (algebraic) and Rectangles (Segment-Segment intersection).
3.  **RL Wrappers**: `LidarNavigationEnv` normalizes the observations (Lidar ranges [0,1], Robot State) and computes rewards based on distance-to-goal and collisions.
4.  **Policy**: We use PPO with a Multi-Input Policy (handling both vector and Lidar inputs).

## 📊 Results


## 📸 Gallery

### Simulation Outputs
![Simulation Screenshot 1](../assets/Screenshot%202025-12-11%20at%2020.25.55.png)
![Simulation Screenshot 2](../assets/Screenshot%202025-12-11%20at%2021.16.02.png)
