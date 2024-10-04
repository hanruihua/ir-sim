## Pypi

```
py -m build
py -m twine upload dist/*
```

## To do list
- [x] Basic framework
- [x] Mobile robot movement
- [x] collision check
- [x] gif generation
- [x] multi robots mode (collision)  
- [x] sensor lidar
- [x] env res
- [x] collision check with discrete samples
- [x] omni directional robots
- [x] Add custom robot model
- [x] Add sensor: gps, odometry
- [x] Add noise (diff)
- [x] line obstacle
- [x] Add subplot 
- [x] Add collision mode
- [x] map obstacle
- [x] Add functions to access obstacles with different types
- [x] Add draw points
- [x] reformulate obstacles and robots by Object class  
- [x] Add the env logger 
- [x] private and public methods and parameters in class
- [x] Add regular event for other obstacles or robots
- [x] Rearrange the framework of obstacles 
- [x] Add function to construct obstacle and robot
- [x] Attribute of the obstacles and robots
- [x] robot description 
- [x] Construct the object base class for the robot and obstacles
- [ ] All the rotation and translation can be represented by the homogeneous transformation matrix
- [ ] transfer the function from the previous version
- [ ] Maze generator
- [ ] real robot size (LIMO, BYD)
- [ ] Using decorator to update
- [ ] code annotation for main class
- [ ] Add the data monitor
- [ ] Add more scenarios (traffic)
- [ ] Add the interface with gym
- [ ] Support the feature of adding or eliminating obstacles by functions
- [ ] Add functions to access obstacles with different types
- [ ] Add more key functions for keyboard control
- [ ] Develop Tools for tackling Data
- [ ] Add scenarios (tasks), car_racing, maze, traffic
- [ ] 3D visualization
- [ ] check whether the object is convex
- [ ] Add synchronization and asynchronization mode
- [ ] Add tf (similar like ROS tf)
- [ ] pytest
- [ ] record and replay path
- [ ] Plot velocity and acceleration
- [ ] Some judgment functions for control
- [ ] Add example yaml files
- [ ] Add some data structure for plot
- [x] Check the dimension of various values and fix the input error, such as state dim, velocity dim. 
- [ ] Test Scenario for the different robot models and planners: pursue and evade, follow, etc.
- [ ] LLM integration
- [ ] organize the functions to calculate A, b, G, h
- [ ] shape tuple -- vertex   refer to ros marker
- [ ] Reformulate the behavior library
- [ ] Add comments for the functions
- [ ] 3D axis for aerial, legged, and arm robots
