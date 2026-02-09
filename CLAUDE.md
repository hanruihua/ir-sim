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
- `env_plot.py` / `env_plot3d.py`: Matplotlib-based visualization

**World Layer** (`irsim/world/`):
- `World`: Core simulation state and collision detection
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
  - `acker`: `dash`
- Group behaviors in `group_behavior.py` and `group_behavior_methods.py`:
  - `orca` (optimal reciprocal collision avoidance) - requires `pyrvo` package

**Path Planners** (`irsim/lib/path_planners/`):
- `a_star.py`: A* grid-based path planning
- `rrt.py`: Rapidly-exploring Random Tree
- `rrt_star.py`: RRT* optimized path planning
- `probabilistic_road_map.py`: PRM path planning

**Sensors** (`irsim/world/sensors/`):
- `lidar2d.py`: 2D LiDAR simulation
- `sensor_factory.py`: Factory for sensor instantiation

**Map** (`irsim/world/map/`):
- `Map`: Grid-based map for navigation and collision detection
- `ObstacleMap`: Obstacle map representation

**GUI** (`irsim/gui/`):
- `keyboard_control.py`: Keyboard-based robot control (requires `pynput`)
- `mouse_control.py`: Mouse-based interaction

### Key Patterns

- **YAML Configuration**: All scenarios defined in human-readable YAML files
- **Factory Pattern**: Objects created via `object_factory.py` from YAML specs
- **Registry Pattern**: Behaviors and sensors registered via decorators for extensibility
- **Geometry via Shapely**: Collision detection uses Shapely library (>=2.0.3)

### Directory Structure

```
irsim/                  # Main package
├── env/                # Environment and visualization
├── world/              # Core simulation components
│   ├── robots/         # Robot kinematics (diff, omni, acker)
│   ├── obstacles/      # Obstacle types (static, dynamic)
│   ├── sensors/        # Sensor implementations (lidar2d)
│   └── map/            # Map and obstacle map
├── lib/                # Algorithms and behaviors
│   ├── behavior/       # Robot behaviors (dash, rvo, orca)
│   ├── algorithm/      # Core algorithms (kinematics, rvo, generation)
│   ├── path_planners/  # Path planning (A*, RRT, RRT*, PRM)
│   └── handler/        # Geometry and kinematics handlers
├── gui/                # Keyboard/mouse controls
├── util/               # Utility functions
└── config/             # Configuration parameters

tests/                  # Pytest test suite
usage/                  # Example YAML configs and scripts
docs/                   # Sphinx documentation (multilingual: en, zh_CN)
```

## Configuration

Ruff is configured in `pyproject.toml` with:
- Line length: 88
- Target: Python 3.10+
- First-party imports: `irsim`

Type checking uses `ty` with custom rule configurations in `pyproject.toml`.

## Dependencies

**Core dependencies**: matplotlib, shapely (>=2.0.3), numpy, pyyaml, imageio, loguru, scipy

**Optional dependencies**:
- `pynput`: Keyboard control (`pip install ir-sim[keyboard]`)
- `pyrvo`: ORCA group behavior (`pip install pyrvo`)
- All extras: `pip install ir-sim[all]`

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

- The documentation is written in markdown format. 
- If you change the english documentation, please also change the chinese documentation.
- If there are code changes, please also update the related documentation.
- documentation files are located in the `docs` directory. The main documentation file is `index.md`. Other documentation files are located in the `docs` directory and its subdirectories.