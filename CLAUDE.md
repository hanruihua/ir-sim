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

# Run tests with coverage
pytest --cov . --cov-report=html

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

**Obstacles** (`irsim/world/obstacles/`):
- `obstacle_static.py`: Static obstacles
- `obstacle_diff.py`: Differential drive dynamic obstacles
- `obstacle_omni.py`: Omnidirectional dynamic obstacles
- `obstacle_acker.py`: Ackermann steering dynamic obstacles

**Behaviors** (`irsim/lib/behavior/`):
- Registry-based system using decorators (`behavior_registry.py`)
- Individual behaviors registered by robot type + behavior name:
  - `diff`: `dash`, `rvo`
  - `omni`: `dash`, `rvo`
  - `omni_angular`: `dash`
  - `acker`: `dash`
- Group behaviors in `group_behavior.py` and `group_behavior_methods.py`:
  - `orca` (optimal reciprocal collision avoidance) - requires `pyrvo` package

**Path Planners** (`irsim/lib/path_planners/`):
- `a_star.py`: A* grid-based path planning
- `rrt.py`: Rapidly-exploring Random Tree
- `rrt_star.py`: RRT* optimized path planning
- `informed_rrt_star.py`: Informed RRT* path planning
- `jps.py`: Jump Point Search (optimized A* variant)
- `probabilistic_road_map.py`: PRM path planning

**Sensors** (`irsim/world/sensors/`):
- `lidar2d.py`: 2D LiDAR simulation
- `sensor_factory.py`: Factory for sensor instantiation

**Map** (`irsim/world/map/`):
- `obstacle_map.py`: Obstacle map representation
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

### Directory Structure

```
irsim/                  # Main package
├── env/                # Environment and visualization (2D/3D)
├── world/              # Core simulation components
│   ├── robots/         # Robot kinematics (diff, omni, acker)
│   ├── obstacles/      # Obstacle types (static, dynamic)
│   ├── sensors/        # Sensor implementations (lidar2d)
│   ├── map/            # Map generators (grid, image, perlin, hm3d)
│   └── description/    # Robot/vehicle visualization assets (PNG)
├── lib/                # Algorithms and behaviors
│   ├── behavior/       # Robot behaviors (dash, rvo, orca)
│   ├── algorithm/      # Core algorithms (kinematics, rvo, generation)
│   ├── path_planners/  # Path planning (A*, RRT, RRT*, Informed RRT*, JPS, PRM)
│   └── handler/        # Geometry and kinematics handlers
├── gui/                # Keyboard/mouse controls
├── util/               # Utility functions
└── config/             # Configuration parameters

tests/                  # Pytest test suite (22 test files)
usage/                  # Example YAML configs and scripts (22 examples)
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

## Release Checklist

When releasing a new version, follow these steps in order:

1. Update the version number in `pyproject.toml`
2. Add a new entry to `docs/source/_static/switcher.json` (mark the new version as `(stable)`, remove that label from the previous one)
3. Summarize the version changes in `changelog.md` (`docs/source/changelog.md` auto-includes it via `{include}`). See the **Changelog Style** section below for entry formatting.
4. Update `CLAUDE.md`'s *Directory Structure* counts if they changed (test files in `tests/`, example scripts in `usage/`)
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
- **Entry format**: one bullet per PR, written as a complete sentence (or two). Lead with the technical change, then state the *motivation/target* — the "why", not just the "what". For example:
  - Good: "Add `random_uniform` sampler with pairwise min-distance. Prevents object overlap in random sampling, and world-derived defaults adapt to the world size/offset so users don't restate bounds per scene."
  - Avoid: "Add `random_uniform` sampler." (what, but no why)
- **PR link**: end every entry with the PR link: `([#NNN](https://github.com/hanruihua/ir-sim/pull/NNN))`.
- **Contributor credit**: for PRs not authored by `hanruihua`, append the GitHub handle after the PR link: `([#NNN](...)) (@username)`.
- **Performance metrics**: quote concrete numbers — measured speedups, coordinate/linestring counts, memory reductions, etc. (e.g., "~48% faster lidar step", "~3× fewer linestrings").
- **Version boundary**: if the current version in `pyproject.toml` has already been tagged/released, open a new `## <next-version>` section at the top instead of appending to the released one.
- **PR and commit alignment**: when opening a PR or writing the commit that introduces a changelog entry, mirror the same summary in the PR description and commit message so the three sources stay in sync.