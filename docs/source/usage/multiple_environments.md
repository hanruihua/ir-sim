# Multiple environments

IR-SIM supports creating and running multiple environments simultaneously. Each environment maintains its own isolated state, including world parameters, objects, and simulation time. This is useful for:

- **Parallel simulations**: Running multiple scenarios simultaneously
- **Comparison studies**: Comparing different algorithms or configurations
- **Training**: Running multiple instances for reinforcement learning

## Creating Multiple Environments

```python
import irsim

# Create two separate environments
env1 = irsim.make('scenario_a.yaml')
env2 = irsim.make('scenario_b.yaml')

# Each environment has its own state
print(f"Env1 robots: {env1.robot_number}")
print(f"Env2 robots: {env2.robot_number}")
```

## Isolated Parameters

Each environment has completely separate parameter instances:

```python
import irsim

env1 = irsim.make('world1.yaml')
env2 = irsim.make('world2.yaml')

# World parameters are isolated
env1.world_param.control_mode = 'keyboard'
print(f"Env1 control mode: {env1.world_param.control_mode}")  # keyboard
print(f"Env2 control mode: {env2.world_param.control_mode}")  # auto (unchanged)

# Simulation time is independent
for _ in range(10):
    env1.step()

for _ in range(5):
    env2.step()

print(f"Env1 time: {env1.time}")  # 1.0 (10 steps * 0.1)
print(f"Env2 time: {env2.time}")  # 0.5 (5 steps * 0.1)
```

## Running Multiple Environments

::::{tab-set}

:::{tab-item} Sequential Execution

```python
import irsim

env1 = irsim.make('scenario_a.yaml', display=True)
env2 = irsim.make('scenario_b.yaml', display=True)

# Run both environments in the same loop
for i in range(500):
    # Step both environments
    env1.step()
    env2.step()

    # Render both (creates two windows)
    env1.render(0.01)
    env2.render(0.01)

    # Check completion independently
    if env1.done() and env2.done():
        break

env1.end()
env2.end()
```
:::

:::{tab-item} Headless Parallel (for Training)

```python
import irsim

# Create multiple headless environments for training
num_envs = 4
envs = [
    irsim.make(f'training_world.yaml', display=False, seed=i)
    for i in range(num_envs)
]

# Run training loop
for episode in range(100):
    # Reset all environments
    for env in envs:
        env.reset()

    # Collect experiences from all environments
    for step in range(1000):
        actions = [get_action(env) for env in envs]  # Your policy

        for env, action in zip(envs, actions):
            env.step(action)

        # Check if any environment is done
        if all(env.done() for env in envs):
            break

# Clean up
for env in envs:
    env.end()
```
:::

::::

## Parameter Isolation Details

Each environment instance creates and binds its own parameter objects:

**`world_param`** - Simulation state and settings:

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| `time` | `float` | `0.0` | Current simulation time |
| `control_mode` | `str` | `"auto"` | Control mode (`"auto"` or `"keyboard"`) |
| `collision_mode` | `str` | `"stop"` | Collision handling mode |
| `step_time` | `float` | `0.1` | Time step duration (seconds) |
| `count` | `int` | `0` | Simulation step counter |

**`env_param`** - Environment objects and utilities:

| Attribute | Type | Description |
|-----------|------|-------------|
| `objects` | `list[ObjectBase]` | List of all objects in the environment |
| `logger` | `EnvLogger` | Logger instance for this environment |
| `GeometryTree` | `STRtree` | Spatial index for collision detection |
| `platform_name` | `str` | Operating system name |

**`path_param`** - File path management:

| Attribute | Type | Description |
|-----------|------|-------------|
| `root_path` | `str` | Path to irsim package directory |
| `ani_buffer_path` | `str` | Path for animation frame buffer |
| `ani_path` | `str` | Path for saved animations |
| `fig_path` | `str` | Path for saved figures |

Objects within each environment reference their own environment's parameters:

:::{note}
When creating multiple environments with `display=True`, each environment opens its own visualization window. For training or batch simulations, use `display=False` to disable rendering and improve performance.
:::
