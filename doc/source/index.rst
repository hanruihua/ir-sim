.. IR-SIM documentation master file, created by
   sphinx-quickstart on Tue Aug  6 00:59:39 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to IR-SIM's documentation!
==================================

**IR-SIM** is an open-source, lightweight robot simulator based on Python, designed for robotics navigation, control, and learning. This simulator provides a simple, user-friendly framework for simulating robots, sensors, and environments, facilitating the development and testing of robotics algorithms with minimal hardware requirements. 

**Features**

- Simulate a wide range of robot platforms with diverse kinematics, sensors, and behaviors. 
- Quickly configure and customize simulation scenarios using straightforward YAML files, with no complex coding required.
- Visualize simulation outcomes in real time for immediate feedback and analysis using a naive visualizer matplotlib.
- Support collision detection and behavior control for each object in the simulation.
The simple demonstrations of the simulator are shown below:

.. image:: https://github.com/user-attachments/assets/5930b088-d400-4943-8ded-853c22eae75b
  :width: 49%
  :alt: Alternative text


.. image:: https://github.com/user-attachments/assets/3257abc1-8bed-40d8-9b51-e5d90b06ee06
  :width: 49%
  :alt: Alternative text

.. image:: https://github.com/user-attachments/assets/7aa809c2-3a44-4377-a22d-728b9dbdf8bc
  :width: 49%
  :alt: Alternative text

.. image:: https://github.com/user-attachments/assets/1cc8a4a6-2f41-4bc9-bc59-a7faff443223
  :width: 49%
  :alt: Alternative text


.. toctree::
   :maxdepth: 2
   :caption: Getting Started:

   get_started/index

.. toctree::
   :maxdepth: 2
   :caption: User Guide:

   usage/index

.. toctree::
   :maxdepth: 2
   :caption: YAML Configuration Syntax:

   yaml_config/index


.. toctree::
   :maxdepth: 2
   :caption: API Documentation:

   api/modules


Cases
--------------

**Academic**

- `rl-rvo-nav(RAL & ICRA2023) <https://github.com/hanruihua/rl_rvo_nav>`_

- `RDA_planner(RAL & IROS2023) <https://github.com/hanruihua/RDA_planner>`_

- `NeuPAN(T-RO 2025) <https://github.com/hanruihua/NeuPAN>`_

**Deep Reinforcement Learning**

- `DRL-robot-navigation-IR-SIM <https://github.com/reiniscimurs/DRL-robot-navigation-IR-SIM>`_


Code Repository
---------------
`IR-SIM <https://github.com/hanruihua/ir-sim>`_