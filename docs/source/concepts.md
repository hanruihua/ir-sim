# Concepts

This page explains the **mental model** behind IR-SIM: how the pieces fit together and why the API looks the way it does. Read it once; the {doc}`User Guide <usage/index>` then shows how to do specific tasks, and the {doc}`Configuration <yaml_config/index>` reference documents every key.

## The big picture

A scene in IR-SIM is a **World** plus a set of **Objects** (robots and obstacles). Objects can carry **sensors** and be driven by **behaviors**. You describe the whole scene declaratively in YAML, create it with `irsim.make()`, and advance it one time step at a time.

```text
            irsim.make("scene.yaml")
                       │
                       ▼
   ┌─────────────────────────────────────────┐
   │            Environment (EnvBase)         │
   │  ┌─────────┐   ┌──────────────────────┐  │
   │  │  World  │   │  Objects              │  │
   │  │ size    │   │   robots ── behavior  │  │
   │  │ time    │   │          └─ sensors   │  │
   │  │ map     │   │   obstacles           │  │
   │  │         │   │                       │  │
   │  └─────────┘   └──────────────────────┘  │
   └─────────────────────────────────────────┘
        step() → render() → done() → end()
```

## The environment

The **environment** (`EnvBase`) is the object you interact with. It owns the world and every object, and it exposes the simulation **lifecycle**:

- **`make()`**: parse the YAML scenario and build the world and objects.
- **`step()`**: advance every object by one time step: resolve external actions or behaviors, integrate object states and geometry, refresh sensors, advance the world clock and fog map, then update arrival and collision status.
- **`render()`**: draw the current state with Matplotlib.
- **`done(mode="all")`**: return whether all robots have arrived or have their stop flag set. Use `mode="any"` to check whether at least one robot is done. A reported collision does not by itself end the loop in every collision mode.
- **`reset()` / `reload()`**: restore objects to their initial states, or rebuild the scene from YAML.
- **`end()`**: close the window and release resources (and write the animation if `save_ani=True`).

This `step → render → done` loop is the heart of every IR-SIM program.

## The world

The **World** holds the dimensions (`width`, `height`), simulation clock (`step_time`), rendering sample interval (`sample_time`), and world offset (`offset`). The environment manages objects and maps, and coordinates collision queries using [Shapely](https://shapely.readthedocs.io/) geometry and a spatial index. Collision status is evaluated at sampled states; it is not a rigid-body contact solver or continuous collision detector.

## Objects: robots and obstacles

Everything placed in the scene is an **object**. Robots and obstacles share the same machinery; the difference is mostly their *role* and whether they are actively controlled. Each object has:

- **State**: its pose, stored as a column vector. Differential and omnidirectional models use `[x, y, theta]`; Ackermann models add steering angle as `[x, y, theta, steer]`. This is what `step()` integrates and what sensors and collisions are computed from.
- **Kinematics**: how a velocity command turns into motion: `diff` (differential drive), `omni` (omnidirectional), `omni_angular`, or `acker` (Ackermann / car-like). An object with no kinematics is *static*.
- **Shape**: `circle`, `rectangle`, `polygon`, `linestring`, or `compound`, used for both collision and drawing. Compound parts have poses relative to the object's local frame; the object's state places the whole shape in the world.
- **Goal**: an optional target pose; arrival is detected within a distance threshold.

### State and action conventions

YAML expresses states and velocities as flat lists. Inside Python, IR-SIM stores
them as column arrays. The meaning of an action depends on the kinematics model:

| Kinematics | State | Action / velocity |
| --- | --- | --- |
| `diff` | `[x, y, theta]` | `[linear, angular]` |
| `omni` | `[x, y, theta]` | Body-frame `[forward, lateral]`; `theta` is preserved |
| `omni_angular` | `[x, y, theta]` | Body-frame `[forward, lateral, yaw_rate]` |
| `acker` | `[x, y, theta, steer]` | `[linear, steering_angle]` in `steer` mode; `[linear, steering_angle_rate]` in `angular` mode (not yaw rate) |

Positions are measured in metres, angles in radians, linear velocities in
metres per second, and angular velocities in radians per second. See
{doc}`Configure robots and obstacles <usage/configure_robots_obstacles>` for
configuration examples and velocity limits.

### Models and numerical accuracy

IR-SIM advances kinematic states from velocity commands. It does not model wheel contact forces, tyre slip, or inertial rigid-body dynamics. The differential-drive update uses forward Euler integration with the heading at the beginning of each step. A smaller time step changes this numerical approximation; it does not add a more detailed physical model. The [interactive kinematics lab](get_started/kinematics.md) shows the equations, coordinate frames, and discretization error together.

For reproducible experiments, record the IR-SIM version, YAML configuration, seed, controller settings, step size, and stopping criteria. Compare the same simulated duration when changing the step size, and report rendering settings and hardware when measuring wall-clock performance.

## Behaviors: deciding what to do

A **behavior** maps an object's state (and, for reactive ones, its neighbours) to a velocity command on every step. The built-ins are `dash` (head straight to the goal), `rvo` (reciprocal velocity obstacles), and `sfm` (social force model), plus the group behavior `orca` (optimal reciprocal collision avoidance). You don't have to use a behavior at all; instead, drive an object yourself by passing a command to `env.step(action)`, or {doc}`register your own behavior <usage/configure_behavior>`.

## Sensors: perceiving the world

**Sensors** are attached to objects and produce measurements from the world each step: a 2D LiDAR, a 2D FMCW LiDAR (range *and* per-beam radial velocity), and a field-of-view detector. Sensors are refreshed **after** objects move, so a reading always reflects the latest world state.

## From YAML to a running scene

IR-SIM is **declarative**. The `world`, `robot`, and `obstacle` blocks in your YAML are turned into `World` and object instances by an internal factory, which is why you rarely write imperative setup code. You *describe* the scene and IR-SIM builds it, so the same scenario file can be shared, version-controlled, and reused across experiments. The {doc}`Configuration <yaml_config/configuration>` reference documents every available key.

## Where to go next

- {doc}`Getting Started <get_started/index>`: install IR-SIM and run your first scene.
- {doc}`User Guide <usage/index>`: task-focused how-to guides.
- {doc}`Configuration <yaml_config/configuration>`: the full YAML schema.
- {doc}`API Reference <api/index>`: classes and methods.
