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
- [ ] All the rotation and translation can be represented by the homogeneous transformation matrix

## 🤖 Robot & Movement
- [x] Omni directional robots
- [x] Add custom robot model
- [x] Robot description
- [x] Check the dimension of various values and fix the input error, such as state dim, velocity dim
- [x] Reformulate the behavior library
- [ ] Real robot size (LIMO, BYD)
- [ ] Provide polygon shape robot
- [ ] 3D rigid body
- [ ] Robotics arm, UAV support
- [ ] Interface with URDF file

## 📡 Sensors & Data
- [x] Sensor lidar
- [x] Add sensor: gps, odometry
- [x] Add noise (diff)
- [ ] Develop Tools for tackling Data. Add the data monitor
- [ ] Record and replay path
- [ ] Analyze the performance (jerks, acceleration, etc.)

## 🗺️ Environment & Obstacles
- [x] Line obstacle
- [x] Map obstacle
- [x] Add functions to access obstacles with different types
- [x] Support the feature of adding or eliminating obstacles by functions
- [x] Add binary occupancy grid map for indoor navigation
- [x] Check whether the object is convex
- [ ] Add functions to access obstacles with different types (refinement needed)
- [ ] Develop a lib for configuration of the shape, refer to rviz marker
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
- [ ] Draw error band (uncertainty) https://matplotlib.org/stable/gallery/lines_bars_and_markers/curve_error_band.html
- [ ] Academic Color
- [ ] Add some data structure for plot
- [ ] Complete color map
- [ ] Add subwindows for the simulation

## 🔧 System & Performance
- [x] Add collision mode
- [x] Add regular event for other obstacles or robots
- [x] Private and public methods and parameters in class
- [x] Add the env logger
- [x] Make the dependency of the package optional
- [ ] Using decorator to update
- [ ] Add synchronization and asynchronization mode
- [ ] Add tf (similar like ROS tf)
- [ ] Rewrite some lib functions by using c++ to improve the efficiency
- [ ] Multiprocess for large scale simulation
- [ ] Make multiple env instances

## 🎮 Control & Interaction
- [x] Env res
- [x] Collision check with discrete samples
- [ ] Add more key functions for keyboard control
- [ ] Assign robot goals by mouse click
- [ ] Some judgment functions for control
- [ ] Complete the reactive collision mode

## 🧪 Testing & Quality
- [x] Pytest
- [x] Organize the test cases
- [x] Improve coverage of the code over 90%
- [ ] Improve coverage of the code over 99%
- [ ] Test Scenario for the different robot models and planners: pursue and evade, follow, etc.

## 📚 Documentation & Examples
- [x] Documentation
- [x] Code annotation for main class
- [x] Add comments for the functions
- [x] Add example yaml files
- [x] Default yaml name (same as python file)
- [x] Reorganize the structure of the readme demonstration
- [x] Argument type hint
- [x] Doc Noise world
- [ ] Doc path manager and change the path
- [ ] Refine documentation

## 🔗 External Interfaces
- [ ] Add the interface with gym
- [ ] Interface with ROS
- [ ] Interface with Pybullet or Gazebo
- [ ] Add the interface with opendrive
- [ ] Modular yaml import so that ir-sim can read the yaml file separately

## 🧠 Advanced Features
- [x] Add ORCA Behavior
- [ ] Add wrapper for ORCA algorithm
- [ ] LLM integration
- [ ] Add various navigation algorithms implemented on the ir-sim

## 🚗 Scenarios & Applications
- [ ] Make_scenarios to generate some common scenarios to test, such as car_racing, maze, traffic (Maze generator)
- [ ] Traffic scenarios

---


