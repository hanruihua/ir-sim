# FAQ & Troubleshooting

Common questions and fixes. If something here doesn't help, please [open an issue](https://github.com/hanruihua/ir-sim/issues).

## Installation

**Which Python versions are supported?**

Python 3.10 and newer.

**How do I enable the optional features?**

Some features need extra dependencies:

- **Keyboard control** (`pynput`): `pip install ir-sim[keyboard]`
- **ORCA group behavior** (`pyrvo`): `pip install pyrvo`
- **Saving videos** (`imageio[ffmpeg]`): included in `pip install ir-sim[all]`
- **Everything**: `pip install ir-sim[all]`

## Display and rendering

**Can I run IR-SIM without a display (server / CI / notebook)?**

Yes. Pass `display=False` to skip the interactive window:

```python
env = irsim.make('world.yaml', display=False)
```

On a fully headless machine (no display at all), also select a non-interactive Matplotlib backend **before** importing IR-SIM:

```python
import matplotlib
matplotlib.use("Agg")
import irsim
```

or run the script under a virtual display: `xvfb-run python your_script.py`.

**How do I make the simulation run faster?**

Render less often than you step: increase `world.sample_time` relative to `step_time`, render every N steps instead of every step, or disable rendering entirely with `display=False` for batch runs.

## Saving animations

**How do I export a GIF or video?**

Set `save_ani=True` in `make()` and call `env.end()` to write the file:

```python
env = irsim.make('world.yaml', save_ani=True)
for i in range(300):
    env.step()
    env.render()
    if env.done():
        break
env.end(ending_time=3)
```

GIF export works out of the box. **Video (`.mp4`) requires ffmpeg** — install it with `pip install imageio[ffmpeg]` (or `pip install ir-sim[all]`). See {doc}`Render and Save Animation <usage/save_animation>`.

## Reproducibility

**How do I get reproducible random scenes?**

Seed IR-SIM's RNG, either through `make()` or `set_seed`:

```python
import irsim
env = irsim.make('world.yaml', seed=42)

# or, equivalently
from irsim.util.random import set_seed
set_seed(42)
```

All built-in randomness (random distributions, random shape generators) routes through this RNG. Custom code using `numpy.random` or Python's `random` should be seeded separately, or switched to IR-SIM's RNG.

## Common errors

**`ModuleNotFoundError: pynput`** — Keyboard control needs `pynput`: `pip install ir-sim[keyboard]`.

**`ModuleNotFoundError: pyrvo`** — The ORCA group behavior needs `pyrvo`: `pip install pyrvo`.

**Saving a video produces nothing** — `ffmpeg` is missing. Install it with `pip install imageio[ffmpeg]`. GIF export does not need ffmpeg.
