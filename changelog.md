## 2.8.0

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

## 2.7.5

- Features:
  - Add features to set the seed in random environment generation for reproducibility. Simply add the seed parameter to the make function (e.g., `env = irsim.make("random_obstacle.yaml", seed=2)`) or set the seed using the `env.set_random_seed` function followed by `env.reload()`. See usage `random_obstacle_seed` for details.
  - Add ORCA example (behavior) in ir-sim, implemented by PyRVO, a Python binding of the classic ORCA algorithm (C++ version, high efficiency). See `usage orca world` for details.

- Bug fixes:
  - Fix issue where sampling time did not work.
  - Fix documentation errors.

## 2.7.4

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

## 2.7.3

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

## 2.7.2

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

## 2.7.1

- Features:
  - Add `name` attribute for the object. It can be configured in the yaml file. The duplicate name check is added to ensure there are no duplicate names.
  - Clearify the `env.step` action order: 1st. keyboard control; 2nd. input action; 3rd. behavior control.
  - Add `env.get_object_by_name` and `env.get_object_by_id` function to get the object by name and id.

- Bug Fixes:
  - Fix error in documentation and function comments.

## ir-sim 2.7.0

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

## ir-sim 2.6.1

- Bug Fixes:
  - Fix documentation error.

- Features:
  - Add WrapTo2Pi function.
  - WrapTo2Pi for lidar and fov angle range.

## ir-sim 2.6.0

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

## ir-sim 2.5.5

- New Features:
  - Add environment pause and resume function. You can pause/resume the environment by pressing `space` key in the keyboard control. see [keyboard control documentation](https://ir-sim.readthedocs.io/en/stable/usage/configure_keyboard_Mouse_control.html) for detail.
  - Add environment title to show current simulation time and robots status. You can also customize the title by setting the `env.set_title` function. see [make env documentation](https://ir-sim.readthedocs.io/en/stable/usage/make_environment.html) for detail.
  - Change the default figure pixel (size) to be 1000x800.
  
- Enhancements:
  - Refine code style, log warning output, and website documentation.


## ir-sim 2.5.4

- Bug Fixes:
  - Fix the pynput import error

## ir-sim 2.5.3

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

## ir-sim 2.5.2

- Bug Fixes:
  - Fix the bug of the clear components.


## ir-sim 2.5.1

- Bug Fixes:
  - Fix the goal plot coordinate bug.
  - Fix grammar, format, and typos in the documentation and comments.
  - Uniform the name of `accer` to `acce` for acceleration parameter.
  - Remove state check warning for obstacles.

- New Features:
  - Add `center` for circular object.

## ir-sim 2.5.0

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

## ir-sim 2.4.4

- Bug Fixes:
  - Fix geometry tree bug
  - Fix 3D plot bug

- New Features:
  - Add rrt star path planner
  - Add neighbor_threshold parameter for the rvo behavior

- Enhancements:
  - Update the distance function to accelerate the rvo algorithm
  - Update the robot image

## ir-sim 2.4.3

- Bug Fixes:
  - Fix lidar offset plot bug.
  
- New Features:
  - Implement several classic path planner, A star, RRT, and PRM adapted from python robotics repo.
  - Add draw quiver functions
  - Add world plot parameters in yaml file.
  - Update requirements.

## ir-sim 2.4.2

- Bug Fixes:
  - Change omni robot default state dim to be 3.
  - Remove the check_collision and arrive property in the env_base.
  - Fix bugs in test cases; Remove some old features (#50).

- New Features:
  - Reorganize the logger functions.
  - Add object heading and orientation (#48).
  - Formulate the geometry tree for the collision check
  - Improve test coverage of this code (#50).
  
## ir-sim 2.4.1

- Bug Fixes:
  - Fix the polygon trail type bug.

- New Features:
  - Make sure the obstacles are valid polygons.
  - Add functions to set the laser color.
  - Env can be initialized without yaml files.
  - Add the render `mode` to select the objects to render. `static`, `dynamic`, `all`.
  - Enhance readme and Documentation.

## ir-sim 2.4.0

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

## ir-sim 2.3.6

- Bug Fixes:
  - Lidar2d sensor pass through the unobstructed objects now, see usage: `05lidar_world` for detail.
  - Fix the matplotlib backend error when running in the headless server.
  - Fix the error of object vertices.
  - Fix the plot parameter name: edgecolor -> trail_edgecolor; linewidth -> trail_linewidth;

- New Features:
  - Add the `obj_linestyle` parameter for the object to set the line style of the object edge. see usage: `05lidar_world` for detail.
  - Update object property comments and API documentation.

## ir-sim 2.3.5

- Bug Fixes:
  - Dimension of init vertices for the polygon object

- New Features:
  - Complete the feature of show_text for the object and add abbr name for the object, run usage: 06multi_objects_world for detail.
  - Add init parameters for the set_goal function.
  - Add add_object, add_objects, delete_object, delete_objects functions for the environment to support adding or eliminating obstacles.
  - Add get_Gh function for the object.
  - Update type hint for the functions and format the code with black.
  
## ir-sim 2.3.4

- Bug Fixes:
  - omni robot dynamics
  - rvo zero division
  - python version compatibility

- New Features:
  - Update random_obstacle_position function to support selection of obstacles and non-overlapping features.
  - Support saving the animation as video.
  
## ir-sim 2.3.2

ir-sim 2.3.2 is a bug fix release with no new features compared to 2.3.1.

## ir-sim 2.3.1

- Update relevant documentation.
- Fix some bugs
- New Features:
  - Add FOV for the object, see usage 15fov_world for detail.
  - Add arguments for function WrapToPi.
  - Add the role selection for the behaviors.

## ir-sim 2.3.0

**Major Version Update:** 

- Publish the documentation for the project.
- Add coverage tests for the project.
- Refactor the kinematics handler and geometry handler to configure the robot and obstacles.
- Reorganize the lidar2d step function to improve its performance.
- Update the comments for the functions.
- Fix some bugs in the project.
- Add a 3D plotting environment.

## ir-sim 2.2.6

- Refactor the behavior library and add a custom behavior interface.
See usage: 13custom_behavior for details.
- Add a default name for YAML files, matching the Python script name.
See usage: 12dynamic_obstacle for details.
- Change the trajectory visualization style.
See usage: 02robot_world for details.
- Reconstruct the save_figure function.
See usage: 07renderw_world for details.
- Fix the 'display' bug in the env.end() function.

## ir-sim 2.2.5

- Convert the GIF file to readme links to reduce the repository size
- Modify the visualization of the arrow for the robot
- Adjust figure size by pixel
- Add reset for world time
- Change the matplotlib backend to TkAgg
- Fix some bugs on the visualization

## ir-sim 2.2.4

- Fix collision avoidance bug for obstacles
- Arrange the kinematics functions
- Add the state_shape and vel_shape for object_base
- Format all python code with black
- Complete the function comments and generate api documentation

## ir-sim 2.2.3

- Rename the module name from ir_sim to irsim, rename the package name from ir_sim to ir-sim
- Add citation for the project
- Refine the comments for the functions
- Configure the readthedoc and sphnix for documentation
- Add the attribute unobstructed for obejcts, see usage: obstace_world for detail
- Fix the state_dim bug

## ir-sim 2.2.0

- Rename the module from ir_sim to irsim, rename the package name from ir_sim to ir-sim
- Add citation for the project
- Refine the comments for the functions


## ir-sim 2.1.4

- Update function comments
- Update the readme
- Add interface to change the object distribution in the environment
- Add Feature of obstacles random in the environment

## ir-sim 2.1.3
- Fix errors in usages
- delete some old files
- Fix some bugs

## ir-sim 2.1.2

- Fix bug for omni dynamics robots
- Fix bug for rvo behavior
- Replace the diff description for the object_base


## ir-sim 2.1.1

- Update the YAML API
- Enhance features in object_base (G, h)
- Expand project development utilities
- Fix some bugs

## ir-sim 2.1.0

Big Version

- Reformulate the whole project framework, all the objects are developed by the object base
- Using shapely to construct the geometry for the robot and obstacles
- Reconstruct the YAML interface
- Add the behavior library for the objects
- Add the object factory to create the objects
- Add env logger to record and print the environment status

## ir-sim 1.1.12

Update the refresh of the show_trail
Add set robot goal method
Fix some bugs


## ir-sim 1.1.11

Add the function to change the edgecolor
Add repeat mkdirs
Fix some bugs

## ir-sim 1.1.10

- Fix some bugs
- Add the collision mode 'unobstructed'
- Add draw box function
- Add radius for obstacle


## ir-sim 1.1.9

- Fix some bugs
- Add features for obstacles.


## ir-sim 1.1.8

- Add map obstacles
- Fix bugs of reset robot
- Add different car models

## ir-sim-v1.1.7

- Fix bugs of pyproject.toml
