---
html_theme.sidebar_secondary.remove: true
---

# Online Playground

Run real IR-SIM Python code in your browser. No Python installation or simulation server is required. Start with a differential-drive robot, change its YAML configuration, and view the simulator's native Matplotlib output.

```{raw} html
<div class="kinematics-lab" data-kinematics-lab data-mode="playground"></div>
```

The page uses the available content width. On wide screens, velocity controls, the plot, and object states sit side by side; smaller windows rearrange the panels automatically. Run and pause stay next to the velocity sliders. Expand **Scene YAML** when you want to edit the configuration; it starts collapsed to leave more room for the experiment.

## A first experiment

1. Click **Launch Python**. The first launch downloads Python and its scientific packages; this can take a while on a slow connection.
2. Click **Run** to start continuous simulation (the default), or use **Step** to advance once. With **Sliders (one diff robot)** selected, adjust speed and yaw rate while running; the latest `[linear_speed, yaw_rate]` is read at the start of each step group, subject to the robot's velocity limits. Select **Behaviors in YAML** to use configured behaviors instead; robots without a behavior stay still.
3. Pause to inspect a state. The time slider replays captured Matplotlib frames; it does not rewind the live environment. Further steps continue from the latest state. Replay retains up to 200 recent frames within a 32 MiB image budget.
4. Expand **Scene YAML**, edit `state`, `shape`, or `world.step_time`, then click **Apply YAML / Reset**. This recreates the environment with the selected seed.
5. Select **Collision & obstacles**, apply its YAML, set yaw rate to zero, and choose a 10-second run. The robot stops when it meets the wall in `collision_mode: stop`.
6. Select **LiDAR & compound shape** and apply its YAML. All built-in scenes default to **Sliders (one diff robot)**; adjust the sliders while running, or select **Behaviors in YAML** for automatic navigation. The L-shaped robot, goal, trajectory, and laser beams are drawn by IR-SIM's existing plotting code. Change `color` or `plot.show_trajectory` in YAML and reset to compare.

Use **Download YAML** to continue locally. **Release runtime** stops the worker and frees its Python environment; snapshots stay visible. If initialization fails or a calculation exceeds its time budget, release and relaunch. Nothing automatically falls back to a JavaScript simulation.

## What runs here?

Each documentation build bundles the IR-SIM source from that checkout. The widget displays its package version, source archive checksum, and pinned Pyodide version. It does not silently install a newer IR-SIM release when you open an older documentation page. The adapter calls `irsim.make(..., display=False)`, `env.step()`, and `env.render()`. Unlike `headless=True`, `display=False` keeps the Matplotlib figure while suppressing the desktop window. The Agg canvas sends PNG frames to the page; JavaScript does not redraw robot geometry, sensors, or trajectories.

This reuses the same plotting logic as local IR-SIM, including YAML plot settings. Fonts and rasterization can differ across Matplotlib versions. The default browser figure is 720 × 576 pixels; `world.plot.figure_pixels` can set each dimension between 240 and 1,000 pixels.

Python runs in a dedicated Web Worker using [Pyodide](https://pyodide.org/en/stable/usage/webworker.html). Computation uses your device, not GitHub or a remote simulation server. Launching downloads runtime assets from jsDelivr and a pinned Python dependency from PyPI; the first launch therefore requires network access.

## Simulation steps and image updates

The default **Continuous** duration runs until you pause, the environment reports completion, or the 2,000-step safety limit is reached. With the default `step_time: 0.2`, that limit allows 400 simulated seconds per reset. Choose **4 s**, **10 s**, or **20 s** before running for a fixed-duration experiment. Pausing prevents further step groups; a group already being computed may finish. Reset after reaching the safety limit to start again.

Continuous runs group complete simulation steps to target at most about 10 image updates per simulated second. Every integration step still calls the original `env.step()` and `env.render()`; neither the configured `step_time` nor sensor/collision calculations are approximated or skipped. Slow devices may take longer than real time. **Step** always advances exactly one simulation step.

The original `world.sample_time` still controls when IR-SIM updates its plot. If it is larger than `step_time`, the Matplotlib frame time can lag the state readout. Replaying a frame shows the stored image and state; it does not recompute the simulation. The object inspector reports the selected object; the figure displays all objects.

## Inspect object states

The **Object inspector** lists every robot and obstacle by name and ID. Select an object to view its role, kinematics, shape, `state`, `velocity`, current `goal`, and the `collision_flag`, `arrive_flag`, `stop_flag`, and `static` values. It updates with each returned simulation snapshot and follows the replay time when paused. Switching objects only changes the readout, not the robot receiving slider commands.

These are snapshots of the object's native attributes, not values recomputed from the image or copied from the command sliders. Arrays keep their model-specific component order; see [state and action conventions](../concepts.md). The display rounds numbers to four decimal places without changing the simulation. A missing goal or kinematics model appears as `—`. The stop flag is the simulator's `stop_flag`, not a test for zero velocity. State time and Matplotlib frame time are shown separately because plot sampling can make them differ.

## Current scope

The interface accepts 2D YAML scenes with up to 10 robots and 50 obstacles, simulation steps from `0.01` to `0.5` seconds, and a 2,000-step budget per reset. Slider commands require a single `diff` robot; other models and multi-robot scenes use **Behaviors in YAML**. Tested examples include `diff`, `omni`, and `acker` with `dash`, compound geometry, trajectories, `lidar2d`, and `fmcw_lidar2d`.

Reusing Matplotlib does **not** make every IR-SIM feature browser-compatible. Desktop keyboard/mouse controls and the Matplotlib window toolbar are not forwarded. This interface does not yet expose file uploads, file/grid maps, 3D projection, arbitrary Python scripts or planner calls, ROS connections, or optional native extensions such as `pyrvo`. Additional features need their own browser integration and tests; importing the simulator alone is not a compatibility guarantee.

The scene and the running environment are local to this tab. Avoid putting secrets in configurations. **Open in Playground** from a tutorial carries its YAML and command settings in the URL fragment; anyone you share that full URL with can read those settings. Reloading does not restore the running simulation.

## Local preview and deployment

Serve the generated files over HTTP(S); module workers cannot run from a `file://` page. From the repository root, after building the documentation:

```bash
python -m http.server 8000 --bind 127.0.0.1 --directory docs/build
```

Open `http://127.0.0.1:8000/html/playground/` for English or `http://127.0.0.1:8000/zh_CN/playground/` for Chinese. The same generated files work on static hosting such as Read the Docs or GitHub Pages; no Python server is needed after deployment.

## Learn with guided experiments

- [From commands to motion](../get_started/kinematics.md): understand the model and reproduce a numerical checkpoint.
- [Concepts](../concepts.md): state, action, coordinate frames, and model assumptions.
- [YAML reference](../yaml_config/configuration.md): look up simulator parameters; the playground's supported subset is listed above.
