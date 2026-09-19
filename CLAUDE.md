# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

IR-SIM is an open-source, Python-based, lightweight robot simulator for navigation, control, and reinforcement learning. It uses YAML-driven configuration for defining robots, sensors, obstacles, and environments with built-in collision detection.

## Development Commands

```bash
# Install dependencies (using uv)
uv sync

# Run all tests
pytest

# Run tests with coverage report (coverage of irsim/ is on by default via pyproject)
pytest --cov-report=html

# Run a single test file
pytest tests/test_kinematics.py

# Run a specific test
pytest tests/test_kinematics.py::test_name

# Linting and formatting
ruff check                    # Check for lint errors
ruff check --fix              # Auto-fix lint errors
ruff format                   # Format code

# Pre-commit hooks (install once)
pre-commit install

# Build documentation
cd docs && make html
```

## Architecture

### Entry Point
`irsim.make('config.yaml')` creates an environment from a YAML configuration file. The main simulation loop uses `env.step()`, `env.render()`, and `env.done()`.

### Core Components

**Environment Layer** (`irsim/env/`):
- `EnvBase` / `EnvBase3D`: Main environment classes handling simulation lifecycle
- `env_config.py`: YAML configuration parsing
- `env_plot.py` / `env_plot3d.py`: Matplotlib-based visualization (2D and 3D)
- `env_logger.py`: Environment logging

**World Layer** (`irsim/world/`):
- `World` / `World3D`: Core simulation state and collision detection (2D and 3D)
- `object_base.py`: Base class for all simulation objects (robots, obstacles)
- `object_factory.py`: Factory pattern for creating objects from YAML
- `object_group.py`: Grouping mechanism for coordinated multi-agent behaviors

**Robot Kinematics** (`irsim/world/robots/`):
- `robot_diff.py`: Differential drive
- `robot_omni.py`: Omnidirectional
- `robot_acker.py`: Ackermann steering (car-like)
- Handlers live in `irsim/lib/handler/kinematics_handler.py`; `PassiveKinematics` is the model of objects configured without kinematics (never drives itself, pushable in `contact` mode with a finite `mass`)

**Obstacles** (`irsim/world/obstacles/`):
- `obstacle_static.py`: Static obstacles
- `obstacle_diff.py`: Differential drive dynamic obstacles
- `obstacle_omni.py`: Omnidirectional dynamic obstacles
- `obstacle_acker.py`: Ackermann steering dynamic obstacles

**Behaviors** (`irsim/lib/behavior/`):
- Registry-based system using decorators (`behavior_registry.py`)
- Individual behaviors registered by robot type + behavior name:
  - `diff`: `dash`, `rvo`, `sfm`
  - `omni`: `dash`, `rvo`, `sfm`
  - `omni_angular`: `dash`
  - `acker`: `dash`
- Group behaviors in `group_behavior.py` and `group_behavior_methods.py`:
  - `orca` (optimal reciprocal collision avoidance) - requires `pyrvo` package
  - `sfm` (vectorized social force model stepping all members from one snapshot, with optional Moussaid 2010 social groups: coherence, repulsion, gaze)
- SFM algorithm implementation: `irsim/lib/algorithm/social_force_model.py` (anisotropic Moussaid-Helbing 2009 variant; `social_force_model` per agent, `SocialForceModelBatch` for a whole crowd)

**Contact / Physics** (`irsim/lib/algorithm/contact.py`):
- `collision_mode: contact` (world YAML) replaces stop-on-collision with engine-like pushing (PhysX rules reduced to `mass`, `friction`, `inertia`, `restitution` and an optional drive lag): after the kinematic step, overlapping pairs are separated along the SAT contact normal. Passive-passive and driven-driven pairs split by inverse mass (inelastic collision; `inv_mass` is 0 for static / `inf`-mass objects); a driven object pushes a passive body without yielding when its `friction_force` (`friction * mass * g`) is at least the summed load of the body and every touching passive body ahead of it in the push direction (`_ContactSolver._load`, per sweep), else it stalls; a passive body pressed on a blocker sticks inside the friction cone (mean of the two `friction`s) and slides outside it; pairs within `REST_TOLERANCE` block without moving. Passive bodies coast under ground friction (`PassiveKinematics.coast`). The world section carries the physics defaults as `WorldParam` fields validated by `World` through `irsim.util.util.check_number`, the one range check also behind object `mass`/`friction`/`inertia`/`restitution` and kinematics `tau`: `gravity` 9.81, `friction` 0.5 (PhysX material), `restitution` 0 (Isaac Lab), `drive_tau` 0 (instant, as Isaac's stiff drives at this step size; opt-in lag); objects read them at use time (`_world_param`) unless they set their own; `obj.passive` and `obj.friction_force` expose the classification. Rotation: `piece_mtv` also returns the contact point (deepest features, midway through the overlap); the split uses generalized inverse masses `w + (r x n)^2 / I` so an off-center push turns a pushable passive body (`inv_inertia`, `inertia` from the shape and `mass` unless set, `gyration` for angular braking); a driven object turns only when a contact makes it yield (stalled or blocked; `yaw_rate_row` on each handler says where the yaw rate goes); passive `action_dim` is 3 (`[vx, vy, yaw_rate]` world-frame). `restitution` (default 0 = Isaac Lab) makes passive bodies separate at e x approach speed after the fold; `contact_step(..., step_time)` fills `Contact.force` = impulse / dt^2 (XPBD) and `obj.contact_force` (a steadily pushed box reports mu m g; a driven object pushing into a contact is capped at its traction, so a stalled robot reports mu m g regardless of dt); `Contact` records keep their deepest push and are kept as `env.contacts` / `obj.contacts` (with `obj.contact_time` / `obj.air_time` ticked each step and mirrored into `ObjectState` as `contact_force`, `contacts`, `contact_time`, `air_time`). Drive lag: `kinematics: {tau: ...}` (`KinematicsHandler.tau`, `obj.drive_tau` falls back to the world's `drive_tau`, default 0) makes `ObjectBase._drive_response` filter commands first-order in contact mode only; the same method caps the drive's speed change per step at `friction * gravity * dt` on the handler's `translation_rows` (wheel grip: a friction-0 robot cannot move, launches and stops take ~3 steps at 0.5) (`_drive_velocity` is the drive's own state, separate from the body velocity the contact fold edits); other modes track instantly
- Per-object `mass` (default `1.0` with kinematics, `inf` without). `ObjectBase._init_motion_state` is the one place that decides `static` (the flag with kinematics; without kinematics, static unless the mass is finite); the factory, plots, env and solver read `obj.static` instead of re-deriving it; `obj.pushable` (not static and finite mass) is the one definition behind `inv_mass` and the color default. Objects without kinematics get a `PassiveKinematics` handler (`kinematics_handler.py`; the `static` name gives the same model) whose `step` keeps the state, whose velocities are world-frame and whose `max_speed` is 0, so `ObjectBase` never branches on the handler kind (`ObjectStatic` only serves `kinematics: {name: static}`); a pushable one defaults to `palette_param.pushable` (Okabe-Ito orange `#E69F00`, print-safe) unless `color` is set, so pushable obstacles stand out from black static ones
- Convex pieces: exact circles, convex polygons, line segments (linestrings, grid-map boundaries); non-convex polygons via `shapely.constrained_delaunay_triangles`; pairs swept in anchored order with blocked-direction projection so chains against walls settle in one sweep
- `ObjectBase.apply_contact_displacement` moves an object and folds the displacement into its velocity; `contact_flag` / `contact_obj` report touches, `collision_flag` stays False for resolved contacts

**Path Planners** (`irsim/lib/path_planners/`):
- `a_star.py`: A* grid-based path planning
- `rrt.py`: Rapidly-exploring Random Tree
- `rrt_star.py`: RRT* optimized path planning
- `informed_rrt_star.py`: Informed RRT* path planning
- `jps.py`: Jump Point Search (optimized A* variant)
- `probabilistic_road_map.py`: PRM path planning

**Sensors** (`irsim/world/sensors/`):
- `lidar2d.py`: 2D LiDAR simulation
- `fmcw_lidar2d.py`: Simplified 2D FMCW LiDAR with per-beam radial velocity
- `sensor_factory.py`: Factory for sensor instantiation

**Map** (`irsim/world/map/`):
- `obstacle_map.py`: Obstacle map representation
- `fog_map.py`: Fog-of-map overlay (`FogMap`, subclass of `Map`) revealed by lidar line of sight or robot field of view
- `grid_map_generator_base.py`: Base grid map generator
- `image_map_generator.py`: Image-based map generation
- `perlin_map_generator.py`: Perlin noise procedural map generation
- `binary_map_generator_hm3d/`: HM3D binary map generator

**GUI** (`irsim/gui/`):
- `keyboard_control.py`: Keyboard-based robot control (requires `pynput`)
- `mouse_control.py`: Mouse-based interaction

### Key Patterns

- **YAML Configuration**: All scenarios defined in human-readable YAML files
- **Factory Pattern**: Objects created via `object_factory.py` from YAML specs
- **Registry Pattern**: Behaviors and sensors registered via decorators for extensibility
- **Geometry via Shapely**: Collision detection uses Shapely library (>=2.1.2)
- **Centralized RNG**: All randomness routes through `irsim.util.random.rng` (a proxy over `numpy.random.Generator`); call `set_seed(seed)` to make runs reproducible
- **Palette param**: every default color is a field of `irsim/config/palette_param.py`, built like `env_param`/`world_param` (dataclass + module proxy). Okabe-Ito based, colour-blind and grayscale-print safe: `robot` `#009E73`, `robot_acker` `#117733`, `obstacle` black, `pushable` orange for pushable kinematics-free bodies, `arrow`, `fov`/`fov_edge`, `lidar`, `laser_highlight`, FMCW velocity colors, `marker`/`path`/`quiver` for the draw helpers, and `cycle` for groups configured with `color: 'cycle'`. Consumers read it when an object or plot is created (`KinematicsHandler.default_color`, dataclass `default_factory`, `None` defaults resolved at call time), so `palette_param.robot = ...` before `irsim.make()` restyles a scene. Mirror any changed default in `docs/source/yaml_config/configuration.md` (HTML tree + entries) and the Chinese catalog

### Directory Structure

```
irsim/                  # Main package
├── env/                # Environment and visualization (2D/3D)
├── world/              # Core simulation components
│   ├── robots/         # Robot kinematics (diff, omni, acker)
│   ├── obstacles/      # Obstacle types (static, dynamic)
│   ├── sensors/        # Sensor implementations (lidar2d, fmcw_lidar2d)
│   ├── map/            # Map generators (grid, image, perlin, hm3d)
│   └── description/    # Robot/vehicle visualization assets (PNG)
├── lib/                # Algorithms and behaviors
│   ├── behavior/       # Robot behaviors (dash, rvo, sfm, orca)
│   ├── algorithm/      # Core algorithms (kinematics, rvo, sfm, contact, generation)
│   ├── path_planners/  # Path planning (A*, RRT, RRT*, Informed RRT*, JPS, PRM)
│   └── handler/        # Geometry and kinematics handlers
├── gui/                # Keyboard/mouse controls
├── util/               # Utility functions
└── config/             # Configuration parameters

tests/                  # Pytest test suite (16 test files)
usage/                  # Example YAML configs and scripts (26 examples)
docs/                   # Sphinx documentation (multilingual: en, zh_CN)
```

## Configuration

Ruff is configured in `pyproject.toml` with:
- Line length: 88
- Target: Python 3.10+
- First-party imports: `irsim`
- Lint rules: F, E, W, I, UP, B, RUF, C4, SIM, ISC, RET, PT
- Ignored: E501 (long lines allowed)

Type checking uses `ty` with custom rule configurations in `pyproject.toml`.

## Dependencies

**Core dependencies**: matplotlib, shapely (>=2.1.2), numpy, pyyaml, imageio, loguru, scipy

**Optional dependencies**:
- `pynput`: Keyboard control (`pip install ir-sim[keyboard]`)
- `pyrvo`: ORCA group behavior (`pip install pyrvo`)
- All extras: `pip install ir-sim[all]` (includes pynput, imageio[ffmpeg], pyrvo)

## Testing Notes

- Tests use pytest with fixtures that auto-close matplotlib figures
- CI runs on Python 3.10-3.14 across Ubuntu and macOS
- Linux CI uses `xvfb-run` for headless display testing

## Git Commit Notes

- Please do not mention claude code in the commit messages and PR messages.
- PR Title Format: `<type>(<scope>): <subject>`

    `<scope>` is optional

    ## Example

    ```
    feat: add hat wobble
    ^--^  ^------------^
    |     |
    |     +-> Summary in present tense.
    |
    +-------> Type: chore, docs, feat, fix, refactor, style, or test.
    ```

    More Examples:

    - `feat`: (new feature for the user, not a new feature for build script)
    - `fix`: (bug fix for the user, not a fix to a build script)
    - `docs`: (changes to the documentation)
    - `style`: (formatting, missing semi colons, etc; no production code change)
    - `refactor`: (refactoring production code, eg. renaming a variable)
    - `test`: (adding missing tests, refactoring tests; no production code change)
    - `chore`: (updating grunt tasks etc; no production code change)

- Make the commit message concise and to the point.
- Each commit should run ruff check and format.

- Before each commit, please run the following commands to check the code:

    ```bash
    ruff check
    ```

    If there are any errors, please fix them before committing.

    If there are any warnings, please ignore them.

## IR-sim documentation notes

- The documentation is built with Sphinx and uses a mix of reStructuredText (`.rst`) and Markdown (`.md`) files.
- If you change the english documentation, please also change the chinese documentation.
- If there are code changes, please also update the related documentation.
- Documentation files are located in the `docs` directory. The main Sphinx entrypoint is `docs/index.rst`, with additional documentation files in `docs` and its subdirectories.

## Note

- You can try to use the uv virtual environment for testing and development: ir-sim/.venv/bin/python

## Citation

- IR-SIM has an accompanying arXiv paper: *IR-SIM: A Lightweight Skill-Native Simulator for Navigation, Learning, and Benchmarking* ([arXiv:2606.08729](https://arxiv.org/abs/2606.08729)).
- The canonical paper citation (BibTeX) lives in the README's `## Citation` section, alongside the arXiv badge in the header.
- `CITATION.cff` at the repo root is the *software* citation (separate from the paper); keep its `version`/`date-released` in sync with releases.

## Release Checklist

When releasing a new version, follow these steps in order:

1. Update the version number in `pyproject.toml`
2. Add a new entry to `docs/source/_static/switcher.json` (mark the new version as `(stable)`, remove that label from the previous one)
3. Summarize the version changes in `changelog.md` (`docs/source/changelog.md` auto-includes it via `{include}`). See the **Changelog Style** section below for entry formatting.
4. Update the *Directory Structure* counts in both `CLAUDE.md` and `AGENTS.md` if they changed (test files in `tests/`, example scripts in `usage/`)
5. Run `uv lock` to update `uv.lock`
   - If your local `uv` is older than the one that produced the checked-in lockfile, a full regen will downgrade the lockfile `revision` field. In that case, either upgrade `uv` and rerun, or manually patch only the `ir-sim` `version` line in `uv.lock` to avoid touching `revision`.
6. Run `ruff check` and `ruff format`; commit formatting changes separately (e.g., `style: apply ruff format`) before the version bump commit
7. Commit the version bump. Confirm with the user before committing
   - Must be on the `main` branch
   - Commit message format: `version bump to v<version>`
8. Create a git tag: `git tag v<version>`

## Changelog Style

Rules for writing entries in `changelog.md`:

- **Scope**: only include changes merged into `main`. Do not list work still on feature branches. Skip dependency-only PRs (`chore(deps)`, `chore(deps-dev)`).
- **Section structure**: use `## <version>` as the top heading, then grouped subsections in this order — `Features`, `Performance`, `Fix`, `Refactor`, `Docs`, `Tests`. Include only the categories that apply.
- **Entry format**: bullets mirror the PR-message style — a bold one-sentence headline, then one to three plain sentences describing the symptom, mechanism, or motivation, ending with the PR link. For example:
  - "**A global seed survives `make()`.** Creating an environment without a seed used to replace the shared generator with a fresh unseeded one, so `set_seed(0)` followed by `irsim.make()` was not reproducible. It is now. ([#365](https://github.com/hanruihua/ir-sim/pull/365))"
- **Read the PR body, not just the title**: a feat PR can carry fix/perf items and a refactor PR can list several fixes; each item becomes its own bullet in the section that matches the item (a fix inside a feat PR goes under `Fix`), all linking the same PR. Multi-bug fix PRs stay flat — one bullet per bug, no grouping parent.
- **PR link**: end every entry with the PR link: `([#NNN](https://github.com/hanruihua/ir-sim/pull/NNN))`.
- **Contributor credit**: for PRs not authored by `hanruihua`, append the GitHub handle after the PR link: `([#NNN](...)) (@username)`.
- **Performance metrics**: quote concrete numbers — measured speedups, coordinate/linestring counts, memory reductions, etc. (e.g., "~48% faster lidar step", "~3× fewer linestrings").
- **Version boundary**: if the current version in `pyproject.toml` has already been tagged/released, open a new `## <next-version>` section at the top instead of appending to the released one.
- **PR and commit alignment**: when opening a PR or writing the commit that introduces a changelog entry, mirror the same summary in the PR description and commit message so the three sources stay in sync.
