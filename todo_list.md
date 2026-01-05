# IR-SIM Todo List

## 🎯 Core Framework
- [x] Basic framework
- [x] Mobile robot movement
- [x] Collision check
- [x] Multi robots mode (collision)
- [x] Reformulate obstacles and robots by Object class
- [x] Construct the object base class for the robot and obstacles
- [x] Rearrange the framework of obstacles
- [x] Add function to construct obstacle and robot
- [x] Attribute of the obstacles and robots
- [x] Formulate the geometry tree for the collision check
- [ ] 3D space
- [ ] Add transformation (like ROS tf)
- [ ] Plugin system 
- [ ] template for custom robots, sensor..

## Reinforcement Learning Project Support 

- [ ] Make the env vectorized to support large scale reinforcement learning in parallel (e.g. gymnasium)

## 🤖 Robot & Movement
- [x] Omni directional robots
- [x] Add custom robot model
- [x] Robot description
- [x] Check the dimension of various values and fix the input error, such as state dim, velocity dim
- [x] Provide polygon shape robot
- [x] Reformulate the behavior library
- [ ] 3D rigid body
- [ ] Robotics arm, UAV support
- [ ] Real robot description

## 📡 Sensors & Data
- [x] Sensor lidar
- [x] Add sensor: gps, odometry
- [x] Add noise (diff)
- [ ] Develop Tools for tackling Data. Add the data monitor for performance analysis (jerks, acceleration, etc.).
- [ ] Record and replay path
- [ ] GPS sensor support (state with noise)

## 🗺️ Environment & Obstacles
- [x] Line obstacle
- [x] Map obstacle
- [x] Add functions to access obstacles with different types
- [x] Support the feature of adding or eliminating obstacles by functions
- [x] Add binary occupancy grid map for indoor navigation
- [x] Check whether the object is convex
- [x] Develop a lib for configuration of the shape, refer to rviz marker
- [ ] Add functions to access obstacles with different types (refinement needed)
- [ ] Use scipy convex hull to generate G and h

## 🎨 Visualization & Plotting
- [x] GIF generation
- [x] Add subplot
- [x] 3D visualization
- [x] Show the text of the object
- [x] Draw points
- [x] Reformulate plot function - Transform-based plotting architecture
- [x] FOV transform fixes for proper field of view visualization
- [x] Lidar2D plotting with matplotlib transforms
- [x] 3D plotting fixes for patches, lines, and other elements
- [x] Consistent plotting API with state/vertices parameters
- [x] Add GUI for the windows
- [ ] Academic Color Map
- [ ] Add subwindows for the visulization
- [ ] Add appearance file for the plot
- [ ] Refine plot function with collection of elements
- [ ] Draw error band (uncertainty) https://matplotlib.org/stable/gallery/lines_bars_and_markers/curve_error_band.html

## 🔧 System & Performance
- [x] Add collision mode
- [x] Add regular event for other obstacles or robots
- [x] Private and public methods and parameters in class
- [x] Add the env logger
- [x] Make the dependency of the package optional
- [x] Env reset function
- [x] Collision check with discrete samples
- [x] Add more key functions for keyboard control (esc, space, r, l, x)
- [x] Assign robot goals by mouse click
- [x] Add reload function for the environment
- [x] moving view of robots.
- [ ] Using decorator for function input checking
- [ ] Add synchronization and asynchronization mode
- [ ] Multi-process for large scale simulation
- [ ] Make multiple env instances
- [ ] accelerate the simulation (collision check, kinematics, etc.) by numba/c++
- [ ] Complete the reactive collision mode
- [ ] YAML schema validation with clear error messages and defaults.


## 🧪 Testing & Quality
- [x] Pytest
- [x] Organize the test cases
- [x] Improve coverage of the code over 90%
- [x] reduce the dependency of the package
- [x] using uv to manage and format the dependency and project
- [ ] Improve coverage of the code over 99%

## 📚 Documentation & Examples
- [x] Documentation
- [x] Code annotation for main class
- [x] Add comments for the functions
- [x] Add example yaml files
- [x] Default yaml name (same as python file)
- [x] Reorganize the structure of the readme demonstration
- [x] Argument type hint
- [x] Doc Noise world
- [x] Refine documentation and the comment of the code function. 
- [x] Doc of status; title; keyboard space
- [x] New style of documentation
- [ ] Add tutorial with more examples.
- [ ] Doc path manager and change the path
- [ ] Generate Logo
- [x] Chinese version

## 🔗 External Interfaces
- [ ] Add the interface with gym
- [ ] Add the interface with ROS
- [ ] Add the interface with Pybullet, Gazebo, Carla
- [ ] Add the interface with opendrive
- [ ] Add the interface with URDF file

## 🧠 Advanced Features
- [x] Add wrapper for ORCA algorithm
- [ ] Add various navigation algorithms implemented on the ir-sim
- [ ] Modular yaml import so that ir-sim can read the yaml file separately
- [ ] Add more behaviors for the objects (orca, pure pursuit, etc.)
- [ ] Benchmark suites with standardized metrics and scoring scripts.

## 🚗 Scenarios & Applications
- [ ] Make_scenarios to generate some common scenarios to test, such as car_racing, maze, traffic (Maze generator)
- [ ] Traffic scenarios (Traffic generator)
- [ ] Maze scenarios (Maze generator)
- [ ] Test Scenario for the different robot models and planners: pursue and evade, follow, etc.

---


