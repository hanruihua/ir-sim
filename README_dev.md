# Intelligent-Robot-Simulator2

A python based robot simulator framework for the intelligent robotics navigation and learning

## Prerequisite

Test platform: Ubuntu20.04, windows10

- Python: >= 3.8
    - numpy  
    - matplotlib 
    - scipy

## Installation

- Install this package by pip:

```
pip install ir_sim
```

- or install manually: 

Clone and install the package

```
git clone https://github.com/hanruihua/ir_sim.git    
cd ir_sim   
pip install -e .  
```

## Usage

The examples are in the ir_sim/usage

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
- [ ] code annotation for main class
- [ ] Support add or eliminates obstacles by functions
- [ ] Add the env logger 
- [ ] using the mouse to select the circle and polygon obstacles
- [ ] Add the data monitor
- [ ] Add more scenarios (traffic)
- [ ] Add the interface with gym
- [ ] render model using matplotlib animation
- [ ] Support the feature of adding or eliminating obstacles by functions
- [ ] Add functions to access obstacles with different types
- [ ] code annotation for main class
- [ ] Add more key functions for keyboard control
- [ ] Add draw points
- [ ] private and public methods and parameters in class
- [ ] Add regular event for other obstacles or robots
- [ ] Rearrange the framework of obstacles  
