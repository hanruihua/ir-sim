# PyTorch wrapper

`TorchWrapper` connects the existing NumPy-based IR-SIM environment to a custom
PyTorch training loop. The simulation still runs on the CPU; the wrapper only
converts actions and observation arrays between NumPy and PyTorch.

## Installation

PyTorch is optional and is imported only when the wrapper is created:

```bash
pip install torch
```

## Basic usage

```python
import irsim
from irsim.wrappers import TorchWrapper

cpu_env = irsim.make("world.yaml", display=False)
env = TorchWrapper(cpu_env, device="cuda", dtype="float32")

state = env.get_robot_state().squeeze(-1)
action = policy(state).unsqueeze(-1)
env.step(action)

env.close()
```

`step()` accepts Tensor actions and moves them to CPU NumPy arrays before calling
the original environment. `get_robot_state()`, `get_lidar_scan()`, and
`get_lidar_offset()` return tensors on the configured device. Floating arrays use
`float32` by default; pass `dtype=None` to preserve their NumPy dtype.

## Multi-robot training

Training code commonly needs one leading robot dimension. The plural getters
collect the same fields that would otherwise require loops over `robot_list`:

```python
states = env.get_robot_states()  # (num_robots, state_dim, 1)
goals = env.get_robot_goals()  # (num_robots, goal_dim, 1)
velocities = env.get_robot_velocities()  # (num_robots, action_dim, 1)
status = env.get_robot_status()

arrived = status["arrive"]  # bool Tensor, (num_robots,)
collided = status["collision"]  # bool Tensor, (num_robots,)
```

Pass `ids=[...]` to select robots. Homogeneous values are stacked by default. If
robots have different state dimensions or some goals are `None`, pass
`stack=False` to keep a list of tensors.

A policy may return actions as `(num_robots, action_dim)` or
`(num_robots, action_dim, 1)`. The wrapper converts either layout to IR-SIM's
per-robot action list:

```python
states = env.get_robot_states().squeeze(-1)
actions = policy(states)
env.step(actions)
```

A single column action with shape `(action_dim, 1)` keeps its original meaning.

## API compatibility

The wrapper preserves the existing IR-SIM lifecycle:

- `step()` and `reset()` keep their original return values.
- `render()`, `done()`, `close()`, and other attributes are delegated to the
  wrapped environment.
- Singular getters do not add a batch dimension. `get_robot_state()` therefore
  keeps `(state_dim, 1)`, while `get_robot_states()` explicitly adds the leading
  robot dimension.
- `env.unwrapped` returns the original IR-SIM environment.

For heterogeneous robot actions, pass a list of tensors just as the NumPy API
expects a list of arrays:

```python
actions = [action_0, action_1]
env.step(actions, action_id=[0, 1])
```

:::{important}
`TorchWrapper` enables GPU policy inference and training, but it does not move
IR-SIM kinematics, collision detection, or sensors to the GPU.
:::

## Wrapper construction

Wrappers are composed explicitly and do not require registration:

```python
env = TorchWrapper(irsim.make("world.yaml", display=False), device="cuda")
```

Explicit construction keeps runtime options such as `device` and `dtype` visible
and allows future wrappers to be nested. Environment registration and wrapper
composition therefore remain separate concerns.
