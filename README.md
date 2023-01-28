# Intelligent Robot Simulator (ir-sim)

A python based robot simulator framework for the intelligent robotics navigation and learning.

Features:  
  * Simple and easy to run with python;  
  * All the models and parameters can be set directly in the yaml configure file;  
  * Environment can be built easily by line, circle model and png images; 
  * Various robot kinematics models: omni-wheel, differential wheel, arckermann; 
  * Support collision check with environment;
  * Equipped with various sensors: Lidar, GPS, Odometry etc.;  
  * Support keyboard controller;
  * Easy to reproduce the robotics algorithms and extend for your own project. 

## Prerequisite

Test platform: Ubuntu20.04, windows10

- Python: >= 3.7
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

The examples are in the [ir_sim/usage](https://github.com/hanruihua/ir_sim/tree/main/ir_sim/usage)

## Author

Han Ruihua  
Contact: hanrh@connect.hku.hk





