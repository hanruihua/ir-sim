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