# Changelog

## 2.10.1

- Fix:
  - Suppress the default velocity arrow on static obstacles by gating the handler-derived `show_arrow` on `not self.static` (regression from v2.9.2's kinematics-handler registry refactor). ([#313](https://github.com/hanruihua/ir-sim/pull/313))

## 2.10.0 (2026-05-24)

Minor release adding two new perception/behavior capabilities — the Social Force Model (SFM) for pedestrian-style crowd avoidance and a simplified 2D FMCW LiDAR sensor with per-beam radial velocity.

- Features:
  - Add simplified 2D FMCW LiDAR sensor with range-and-velocity returns and a `22fmcw_lidar_world` usage example. ([#293](https://github.com/hanruihua/ir-sim/pull/293)) ([@KevinLADLee](https://github.com/KevinLADLee))
  - Add Social Force Model (SFM) behavior for `diff` and `omni` kinematics (anisotropic Moussaid-Helbing 2009 variant) with a `23sfm_world` cross-corridor usage example. ([#307](https://github.com/hanruihua/ir-sim/pull/307))

- Performance:
  - Cache `velocity_xy`, `rvo_neighbor_state`, and `rvo_line_segments` per-tick on `ObjectBase` — ~4× faster SFM/RVO step. ([#307](https://github.com/hanruihua/ir-sim/pull/307))

- Docs:
  - Add release dates to all version headings in `changelog.md`. ([#288](https://github.com/hanruihua/ir-sim/pull/288))
  - Refine the YAML configuration reference and keyboard/mouse control docs for clarity (en + zh_CN). ([#296](https://github.com/hanruihua/ir-sim/pull/296))
  - Document the `sfm` behavior alongside `rvo` (en + zh_CN). ([#307](https://github.com/hanruihua/ir-sim/pull/307))

- Tests:
  - Add `tests/test_sfm.py` covering the SFM algorithm and behavior registration. ([#307](https://github.com/hanruihua/ir-sim/pull/307))

## 2.9.4 (2026-04-21)

- Features:
  - Add `omni_angular` kinematics with 3-DOF body-frame control `[forward, lateral, yaw_rate]`, and refactor `omni` to body-frame `[forward, lateral]`. Body-frame velocities match how physical omni platforms are commanded, and `omni_angular` enables independent yaw control for holonomic/swerve use cases. Keyboard control is redesigned around the new axes. ([#270](https://github.com/hanruihua/ir-sim/pull/270))
  - Add `env.reset(random=True)` to re-sample randomized scenes from the cached YAML parse. Lets you quickly restart with a fresh random scene (new positions, new random shapes) without re-reading the YAML file from disk, which is useful for RL rollouts and batched experiments. ([#283](https://github.com/hanruihua/ir-sim/pull/283))
  - Add `random_uniform` sampler with pairwise min-distance, and world-derived defaults for `random` and `circle` distributions. The pairwise min-distance prevents object overlap in random sampling, and world-derived defaults make the sampling ranges adapt automatically to the world size/offset so users don't have to restate bounds per scene. ([#283](https://github.com/hanruihua/ir-sim/pull/283))
  - Add `env.refresh()` and `ObjectBase.refresh()` to sync geometry, sensors, and collision tree without advancing the simulation. Useful after mutating object states directly (e.g. `robot.set_state(...)`) so sensors and collision data are up to date before the next `env.step()`. ([#284](https://github.com/hanruihua/ir-sim/pull/284))

- Performance:
  - Replace circle-buffer obstacle-map geometry with vectorized `shapely.box` + `unary_union`. Box cells tile the grid perfectly — ~3× fewer linestrings, ~24× fewer coordinates, ~3.8× faster lidar step, and proportionally lighter obstacle-map collision queries — and close diagonal gaps in the old circle buffer that previously let lidar rays leak through walls. ([#276](https://github.com/hanruihua/ir-sim/pull/276))

- Fix:
  - Reset `ObjectBase.id_iter` at `EnvBase.__init__` so each environment starts object IDs at 0. Previously the class-level counter persisted across environments, causing index mismatches when multiple environments were created sequentially. ([#277](https://github.com/hanruihua/ir-sim/pull/277))
  - Preserve initial velocity across `env.reset()` so `set_velocity(init=True)` takes effect. Previously an internal zero-action step inside `reset` clobbered `_velocity` right after it was restored, so users could not set a non-zero starting velocity. ([#284](https://github.com/hanruihua/ir-sim/pull/284))

- Docs:
  - Fix broken cross-references across usage guides and the YAML configuration reference (Markdown `[Name](#irsim.path)` links replaced with Sphinx `{py:class/meth/func}` roles), and update omni velocity description to body-frame throughout EN + zh_CN. ([#278](https://github.com/hanruihua/ir-sim/pull/278))

## 2.9.3 (2026-04-06)

- Features:
  - Add loop mode for waypoint navigation behavior. ([#266](https://github.com/hanruihua/ir-sim/pull/266))
  - Add line obstacle support to RVO algorithm. ([#250](https://github.com/hanruihua/ir-sim/pull/250))
  - Add CBF-QP and collision-cone usage examples. ([#258](https://github.com/hanruihua/ir-sim/pull/258)) ([@Lawliet9666](https://github.com/Lawliet9666))

- Performance:
  - Vectorize lidar range/origin filter and skip `union_all`, ~48% faster lidar step on map-based arenas. ([#257](https://github.com/hanruihua/ir-sim/pull/257)) ([@williamleong](https://github.com/williamleong))
  - Use STRtree intersects predicate for lidar map segment filtering, ~26% faster lidar step on PNG-map arenas. ([#255](https://github.com/hanruihua/ir-sim/pull/255)) ([@williamleong](https://github.com/williamleong))
  - Cache geometry validity and streamline lidar map ray intersection, ~18% faster lidar step on PNG-map arenas. ([#254](https://github.com/hanruihua/ir-sim/pull/254)) ([@williamleong](https://github.com/williamleong))

- Fix:
  - Prevent infinite loop in `WrapToPi`/`WrapTo2Pi` for non-finite inputs. ([#269](https://github.com/hanruihua/ir-sim/pull/269))

- Refactor:
  - Simplify `file_check` and `find_file` in util. ([#256](https://github.com/hanruihua/ir-sim/pull/256))

## 2.9.2 (2026-03-16)

- Features:
  - Add `env.create_robot()` and `env.create_obstacle()` for programmatic object creation, and `env.add_object()` / `env.add_objects()` for adding them at runtime. ([#251](https://github.com/hanruihua/ir-sim/pull/251))
  - Add `check_arrive()` method to check if an object has reached a given goal, and initialize plot for dynamically added objects. ([#240](https://github.com/hanruihua/ir-sim/pull/240))
  - Add `set_text()` and `set_goal_text()` API for setting custom text on objects and goals. ([#239](https://github.com/hanruihua/ir-sim/pull/239))
  - Add typo detection and suggestions for invalid YAML configuration keys. ([#227](https://github.com/hanruihua/ir-sim/pull/227))

- Refactor:
  - Centralize kinematics metadata (default color, state_dim, description) in the handler registry to make the kinematics be added more easily. ([#237](https://github.com/hanruihua/ir-sim/pull/237))

- Fix:
  - Improve robustness of save animation by reading and saving frames one by one instead of loading all images into memory. ([#230](https://github.com/hanruihua/ir-sim/pull/230)) ([@williamleong](https://github.com/williamleong))

- Docs:
  - Add "Dynamic Object Management" section to the Make Environment documentation. ([#251](https://github.com/hanruihua/ir-sim/pull/251))
  - Fix and update Chinese translations.

## 2.9.1 (2026-02-16)

- Features:
  - Add Perlin noise and image-based grid map generators, new JPS (Jump Point Search) and Informed RRT* path planners, and refactor existing planners (A*, RRT, RRT*, PRM) to use the `EnvGridMap` protocol. #215 ([@KevinLADLee](https://github.com/KevinLADLee))
  - Drop Python 3.9 support, add Python 3.14 support. Raise minimum Python version to 3.10, modernize type annotations (`Optional`/`Union` to `X | Y` syntax). #216
  - Add optional `reload` parameter to `set_random_seed()` that regenerates random obstacles with the new seed when set to `True`. #210
  - Default animation filename to the world name when not explicitly specified. #221

- Fix:
  - Add grid-based collision detection for obstacle maps using grid array lookup for faster detection, cache STRtree in `ObstacleMap`. #205

- Refactor:
  - Add input validation decorators (`validate_shape`, `validate_length`, `ensure_column_vector`, `ensure_numpy`) for kinematics and utility functions. #209

- Docs:
  - Refine README. #222
  - Add grid map configuration and path planning documentation. #215

## 2.9.0 (2026-01-26)

This version adds multi-environment support with instance-based parameters, allowing multiple independent simulation environments to run simultaneously. It also introduces 1D ToF sensor support and keyboard improvements.

- Features:
  - Add 1D ToF (Time-of-Flight) sensor support by setting lidar2d sensor `number: 1` and `angle_range: 0`. ([#200](https://github.com/hanruihua/ir-sim/pull/200))
  - Add multi-environment keyboard switching support. Only one environment responds to keyboard input at a time; switching occurs via mouse click or focus events. ([#192](https://github.com/hanruihua/ir-sim/pull/192))
  - Add `y` key to toggle display render window on/off. ([#194](https://github.com/hanruihua/ir-sim/pull/194))

- Refactor:
  - Environment parameters (`env_param`, `world_param`, `path_param`) are now instance-based rather than global, enabling proper multi-environment support. ([#191](https://github.com/hanruihua/ir-sim/pull/191))

- Fix:
  - Fix `robot` property to raise `IndexError` with clear message when no robots exist. ([#188](https://github.com/hanruihua/ir-sim/pull/188))
  - Fix `save_figure` to handle filenames with multiple dots correctly. ([#188](https://github.com/hanruihua/ir-sim/pull/188))
  - Fix type annotations for `get_group_by_name`, `get_obstacle_info_list`, `get_robot_info_list`, and `random_obstacle_position`. ([#188](https://github.com/hanruihua/ir-sim/pull/188))
  - Fix `set_ax_viewpoint` to handle `None` objects parameter. ([#188](https://github.com/hanruihua/ir-sim/pull/188))
  - Add `NotImplementedError` for 3D state generation and uniform distribution (not yet implemented). ([#188](https://github.com/hanruihua/ir-sim/pull/188))

- Docs:
  - Add changelog and contributing pages to documentation. ([#203](https://github.com/hanruihua/ir-sim/pull/203))
  - Improve custom behavior tutorial and examples. ([#193](https://github.com/hanruihua/ir-sim/pull/193))
  - Fix formatting issues in documentation. ([#190](https://github.com/hanruihua/ir-sim/pull/190))

- Tests:
  - Refactor and improve test coverage from 94% to 97%. ([#189](https://github.com/hanruihua/ir-sim/pull/189))
  - Add tests for multi-env keyboard switching and display toggle.

## 2.8.2 (2026-01-05)

- Features #183:
  - Add `group_name` YAML parameter to help user manage the objects in the same group.
  - Add `show_goal_text` YAML parameter to decide whether to show the goal text on the plot.
  - Add `get_group_by_name` function to get the objects in the same group by the group name.
  - Update related documentation
  
- Fix:
  - Fix Group ID parameter conflict issue #180 #183
  - Fix group behavior warning issue e8ca7df

## 2.8.1 (2025-12-13)

Hotfix: Fix the autoapi extension issue in the documentation.

## 2.8.0 (2025-12-12)

This version add the feature of `group_behavior` to support group-level behavior for all objects within the same group, which is more efficient for coordinated behaviors (like swarm or crowd simulation) as it computes actions for all members in a single step. The chinese (中文) version documentation is also supported.

- Features:
  - Add `group_behavior` YAML parameter to support group-level behavior for all objects within the same group.
  - Integrate `GroupBehavior` class to handle group-level logic, allowing for both function-based and class-based behaviors.
  - Add ORCA (Optimal Reciprocal Collision Avoidance) behavior support implemented by `pyrvo` library, this behavior can be configured in the YAML file.
  - Update `behavior_registry` to support registration of behavior classes and group behavior classes using decorators (`@register_behavior_class`, `@register_group_behavior_class`).
  - Add `viewpoint` YAML parameter for rendering the plot with the viewpoint of the object.

- Docs:
  - Support chinese version documentation.
  - Update documentation for `group_behavior` configuration, including ORCA parameters.

- Fix:
  - rng random in generate_polygon function
  - Group index increment issue

## 2.7.5 (2025-10-26)

- Features:
  - Add features to set the seed in random environment generation for reproducibility. Simply add the seed parameter to the make function (e.g., `env = irsim.make("random_obstacle.yaml", seed=2)`) or set the seed using the `env.set_random_seed` function followed by `env.reload()`. See usage `random_obstacle_seed` for details.
  - Add ORCA example (behavior) in ir-sim, implemented by PyRVO, a Python binding of the classic ORCA algorithm (C++ version, high efficiency). See `usage orca world` for details.

- Bug fixes:
  - Fix issue where sampling time did not work.
  - Fix documentation errors.

## 2.7.4 (2025-10-06)

- Features:
  - Add state normalization and an input-checking decorator function.
  - Add new key `F5` to debug the environment. In debug mode, the environment pauses and waits for the next F5 to continue; press Space to exit debug mode.
  - Add new key `v` to save the current figure.
  - Add flags for reload, debug, and quit.

- Bug fixes:
  - Fix keyboard issues.
  - Fix goal plot issues.

- Style:
  - Organize plot function for patch plot.
  - Organize logger info.

## 2.7.3 (2025-09-20)

- Features:
  - Switch the default keyboard backend to `pynput` (global keyboard hook), with automatic fallback to Matplotlib (`mpl`) when `pynput` is unavailable. Note: `mpl` can introduce delays when plotting many objects.
  - Parity for the `pynput` keyboard backend with `mpl`:
    1) Keyboard input is handled only when the figure window is focused.
    2) Add a `global_hook` option in the keyboard config to allow input when the figure window is unfocused.
    3) Implement `reload` and `quit` for the `pynput` backend.
  - Add an environment-level `quit` function.
  - Rename keyboard handlers in `keyboard_control.py`: `_on_release` → `_on_pynput_release`, `_on_press` → `_on_pynput_press`.

- Bug fixes:
  - Correct goal plot alpha handling.
  - Set default sample time to match step time.
  - Fix random obstacle plotting.
  - Fix GUI YAML example in the usage docs.
  - Resolve LiDAR step timing for dynamic objects (LiDAR updates after all object poses are updated).
  - Fix keyboard issue on macOS.

- Docs:
  - Clarify sensor update order.
  - Document keyboard control using the `pynput` backend.

## 2.7.2 (2025-08-31)

- Features:
  - Add a new gui section to the YAML configuration for keyboard and mouse controls. see [gui configuration](https://ir-sim.readthedocs.io/en/latest/yaml_config/configuration.html#gui-configuration) 
  
  - Change the default keyboard backend to Matplotlib (`mpl`), which uses figure window key events. The `mpl` backend is active when the Matplotlib figure window is focused. The `pynput` backend, which provides a global keyboard hook, remains supported and can be selected in YAML.
  
  - Add `env.reload()` to reload the environment on the fly, enabling YAML changes to be applied without closing the Matplotlib figure window.
  
  - Add hotkeys to configure and update environment settings, available in both `auto` and `keyboard` control modes:
    - `r`: Reset the environment
    - `space`: Pause/Resume the environment
    - `esc`: Quit the environment
    - `x`: Switch between keyboard and auto control modes
    - `l`: Reload the environment and apply the updated YAML without closing the Matplotlib figure window
  
  - update corresponding documentation and usage.

## 2.7.1 (2025-08-25)

- Features:
  - Add `name` attribute for the object. It can be configured in the yaml file. The duplicate name check is added to ensure there are no duplicate names.
  - Clearify the `env.step` action order: 1st. keyboard control; 2nd. input action; 3rd. behavior control.
  - Add `env.get_object_by_name` and `env.get_object_by_id` function to get the object by name and id.

- Bug Fixes:
  - Fix error in documentation and function comments.

## ir-sim 2.7.0 (2025-08-10)

This version improves the CI pipeline, GitHub Actions workflows, linting, and formatting to ensure consistent style and faster checks, with no API changes.

- Workflow improvements:
  - Adopted `uv` to manage the project and dependencies; pyproject.toml is now uv-managed.
  - Using `uv.lock` to lock the dependencies.
  - Using `Ruff` for linting and formatting (replacing `Black`).
  - Added type hint and Using `ty` for type checking.
  - Added `pre-commit` configuration (.pre-commit-config.yaml) to run linting and formatting before commits.
  - Reorganized GitHub Actions workflows to test the API and enforce code style.
  - Added issue templates; dependabot.yml; Contributing;

- Bug fixes:
  - Fixed documentation errors.
  - Fixed 3D plot title.

- Folder structure:
  - Renamed `doc/` to `docs/`.
  - Moved `irsim/usage/` to `usage/` at the repository root.

## ir-sim 2.6.1 (2025-07-28)

- Bug Fixes:
  - Fix documentation error.

- Features:
  - Add WrapTo2Pi function.
  - WrapTo2Pi for lidar and fov angle range.

## ir-sim 2.6.0 (2025-07-21)

This version is a major update of the documentation website. The new website is more user-friendly and easier to navigate the parameters usage and the API.

- Main Features
  - Add version switcher to the documentation.
  - Refine the documentation style.
  - Refine the comments and code style.

- Bug Fixes:
  - Extract item from ndarray
  - Fix goal orientation and traj keep length
  - Fix gif generation bug

- API Changes:
  - Rename `global_param` folder to be `config`. All the call of the `global_param` should be replaced to `config`.
  - Rename `keep_length` to `keep_traj_length` and `keep_trail_length`.

## ir-sim 2.5.5 (2025-06-29)

- New Features:
  - Add environment pause and resume function. You can pause/resume the environment by pressing `space` key in the keyboard control. see [keyboard control documentation](https://ir-sim.readthedocs.io/en/stable/usage/configure_keyboard_Mouse_control.html) for detail.
  - Add environment title to show current simulation time and robots status. You can also customize the title by setting the `env.set_title` function. see [make env documentation](https://ir-sim.readthedocs.io/en/stable/usage/make_environment.html) for detail.
  - Change the default figure pixel (size) to be 1000x800.
  
- Enhancements:
  - Refine code style, log warning output, and website documentation.


## ir-sim 2.5.4 (2025-06-16)

- Bug Fixes:
  - Fix the pynput import error

## ir-sim 2.5.3 (2025-06-15)

- New Features:
  - Add GUI module, including keyboard and mouse control. 
  - Add mouse control documentation.
  - Make the object default goal to be None.

- Bug Fixes:
  - Fix the backend error for macos system.
  - Fix zorder issue for the object plot.
  - Fix the object collision check issue. 

- API Changes:
  - Add `mouse_left_pos`, `mouse_right_pos`, and `mouse_pos` attributes to the environment for mouse control.
  - Move keyboard_control.py to the gui folder. Thus the api of the keyboard is changed, such as from `env.alt_flag` to `env.keyboard.alt_flag`. 

## ir-sim 2.5.2 (2025-06-02)

- Bug Fixes:
  - Fix the bug of the clear components.


## ir-sim 2.5.1 (2025-06-02)

- Bug Fixes:
  - Fix the goal plot coordinate bug.
  - Fix grammar, format, and typos in the documentation and comments.
  - Uniform the name of `accer` to `acce` for acceleration parameter.
  - Remove state check warning for obstacles.

- New Features:
  - Add `center` for circular object.

## ir-sim 2.5.0 (2025-05-26)

This version improves performance (approximately 40% speed improvement) by refactoring the object plot function and the geometry transform operation for large environments.

- New Features:
  - Refactor the object plot function using Matplotlib patch transforms.  
  - Add new object plot features and API.  
  - Refactor the geometry-transform operation.  
  - Improve test coverage.  
  - Add a set-laser-color function.  
  - Refine documentation and the to-do list.  
  - Add a UV lock file for the project.  

- Bug Fixes:
  - Fix bugs in object plot.  
  - Rename `init_vertices` to `original_vertices`.  
  - Rename `init_geometry` to `original_geometry`.  
  - Fix warning messages.

## ir-sim 2.4.4 (2025-05-06)

- Bug Fixes:
  - Fix geometry tree bug
  - Fix 3D plot bug

- New Features:
  - Add rrt star path planner
  - Add neighbor_threshold parameter for the rvo behavior

- Enhancements:
  - Update the distance function to accelerate the rvo algorithm
  - Update the robot image

## ir-sim 2.4.3 (2025-04-30)

- Bug Fixes:
  - Fix lidar offset plot bug.
  
- New Features:
  - Implement several classic path planner, A star, RRT, and PRM adapted from python robotics repo.
  - Add draw quiver functions
  - Add world plot parameters in yaml file.
  - Update requirements.

## ir-sim 2.4.2 (2025-04-19)

- Bug Fixes:
  - Change omni robot default state dim to be 3.
  - Remove the check_collision and arrive property in the env_base.
  - Fix bugs in test cases; Remove some old features (#50).

- New Features:
  - Reorganize the logger functions.
  - Add object heading and orientation (#48).
  - Formulate the geometry tree for the collision check
  - Improve test coverage of this code (#50).
  
## ir-sim 2.4.1 (2025-04-02)

- Bug Fixes:
  - Fix the polygon trail type bug.

- New Features:
  - Make sure the obstacles are valid polygons.
  - Add functions to set the laser color.
  - Env can be initialized without yaml files.
  - Add the render `mode` to select the objects to render. `static`, `dynamic`, `all`.
  - Enhance readme and Documentation.

## ir-sim 2.4.0 (2025-03-20)

- New Features:
  - Add binary map generated from the 3D scene dataset (Hm3d).
  - Accelerate collision check for larger binary map.
  - Support Multiple Goal setting.
  - Add the requirement files. 
  - Set random goals call for objects.
  
- Bug Fixes:
  - Fix the bug of the collision mode, unobstructed obstacles will not be considered in the collision check.
  - Fix the typo in readme and documentation.
  - Fix the error in log message.
  - Fix the multi-object keyboard control error. 

## ir-sim 2.3.6 (2025-03-12)

- Bug Fixes:
  - Lidar2d sensor pass through the unobstructed objects now, see usage: `05lidar_world` for detail.
  - Fix the matplotlib backend error when running in the headless server.
  - Fix the error of object vertices.
  - Fix the plot parameter name: edgecolor -> trail_edgecolor; linewidth -> trail_linewidth;

- New Features:
  - Add the `obj_linestyle` parameter for the object to set the line style of the object edge. see usage: `05lidar_world` for detail.
  - Update object property comments and API documentation.

## ir-sim 2.3.5 (2025-02-27)

- Bug Fixes:
  - Dimension of init vertices for the polygon object

- New Features:
  - Complete the feature of show_text for the object and add abbr name for the object, run usage: 06multi_objects_world for detail.
  - Add init parameters for the set_goal function.
  - Add add_object, add_objects, delete_object, delete_objects functions for the environment to support adding or eliminating obstacles.
  - Add get_Gh function for the object.
  - Update type hint for the functions and format the code with black.
  
## ir-sim 2.3.4 (2025-02-18)

- Bug Fixes:
  - omni robot dynamics
  - rvo zero division
  - python version compatibility

- New Features:
  - Update random_obstacle_position function to support selection of obstacles and non-overlapping features.
  - Support saving the animation as video.
  
## ir-sim 2.3.2 (2025-01-28)

ir-sim 2.3.2 is a bug fix release with no new features compared to 2.3.1.

## ir-sim 2.3.1 (2025-01-26)

- Update relevant documentation.
- Fix some bugs
- New Features:
  - Add FOV for the object, see usage 15fov_world for detail.
  - Add arguments for function WrapToPi.
  - Add the role selection for the behaviors.

## ir-sim 2.3.0 (2024-12-22)

**Major Version Update:** 

- Publish the documentation for the project.
- Add coverage tests for the project.
- Refactor the kinematics handler and geometry handler to configure the robot and obstacles.
- Reorganize the lidar2d step function to improve its performance.
- Update the comments for the functions.
- Fix some bugs in the project.
- Add a 3D plotting environment.

## ir-sim 2.2.6 (2024-11-22)

- Refactor the behavior library and add a custom behavior interface.
See usage: 13custom_behavior for details.
- Add a default name for YAML files, matching the Python script name.
See usage: 12dynamic_obstacle for details.
- Change the trajectory visualization style.
See usage: 02robot_world for details.
- Reconstruct the save_figure function.
See usage: 07renderw_world for details.
- Fix the 'display' bug in the env.end() function.

## ir-sim 2.2.5 (2024-10-17)

- Convert the GIF file to readme links to reduce the repository size
- Modify the visualization of the arrow for the robot
- Adjust figure size by pixel
- Add reset for world time
- Change the matplotlib backend to TkAgg
- Fix some bugs on the visualization

## ir-sim 2.2.4 (2024-09-17)

- Fix collision avoidance bug for obstacles
- Arrange the kinematics functions
- Add the state_shape and vel_shape for object_base
- Format all python code with black
- Complete the function comments and generate api documentation

## ir-sim 2.2.3 (2024-09-01)

- Rename the module name from ir_sim to irsim, rename the package name from ir_sim to ir-sim
- Add citation for the project
- Refine the comments for the functions
- Configure the readthedoc and sphnix for documentation
- Add the attribute unobstructed for obejcts, see usage: obstace_world for detail
- Fix the state_dim bug

## ir-sim 2.2.0 (2024-08-30)

- Rename the module from ir_sim to irsim, rename the package name from ir_sim to ir-sim
- Add citation for the project
- Refine the comments for the functions


## ir-sim 2.1.4 (2024-08-19)

- Update function comments
- Update the readme
- Add interface to change the object distribution in the environment
- Add Feature of obstacles random in the environment

## ir-sim 2.1.3 (2024-08-11)
- Fix errors in usages
- delete some old files
- Fix some bugs

## ir-sim 2.1.2 (2024-08-04)

- Fix bug for omni dynamics robots
- Fix bug for rvo behavior
- Replace the diff description for the object_base


## ir-sim 2.1.1 (2024-07-15)

- Update the YAML API
- Enhance features in object_base (G, h)
- Expand project development utilities
- Fix some bugs

## ir-sim 2.1.0 (2024-03-29)

Big Version

- Reformulate the whole project framework, all the objects are developed by the object base
- Using shapely to construct the geometry for the robot and obstacles
- Reconstruct the YAML interface
- Add the behavior library for the objects
- Add the object factory to create the objects
- Add env logger to record and print the environment status

## ir-sim 1.1.12 (2023-11-02)

Update the refresh of the show_trail
Add set robot goal method
Fix some bugs


## ir-sim 1.1.11 (2023-04-28)

Add the function to change the edgecolor
Add repeat mkdirs
Fix some bugs

## ir-sim 1.1.10 (2023-04-12)

- Fix some bugs
- Add the collision mode 'unobstructed'
- Add draw box function
- Add radius for obstacle


## ir-sim 1.1.9 (2023-03-20)

- Fix some bugs
- Add features for obstacles.


## ir-sim 1.1.8 (2023-02-27)

- Add map obstacles
- Fix bugs of reset robot
- Add different car models

## ir-sim-v1.1.7 (2023-01-16)

- Fix bugs of pyproject.toml
