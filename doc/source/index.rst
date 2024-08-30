.. IR-SIM documentation master file, created by
   sphinx-quickstart on Tue Aug  6 00:59:39 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to IR-SIM's documentation!
==================================

**Intelligent Robotics Simulation (IR-SIM)** is a Python based lightweight 2D robotics simulator that provides a simple and easy-to-use interface for simulating robot behaviors in a 2D environment. The simulator is designed to be used for educational purposes and research in the field of robotics. 

It provides the following features:
  - A versatile and easy-to-use framework for simulating a variety of robot platforms with kinematics and sensors. 
  - Customizable configurations and parameters using yaml files.
  - Real-time visualization of simulation outcomes.
  - Ideal for developing and testing algorithms related to robot navigation, motion planning, reinforcement learning.


.. toctree::
   :maxdepth: 2
   :caption: Getting Started:

   install
   quick_start
   configuration
   usage

.. toctree::
   :maxdepth: 2
   :caption: API Documentation:

   modules


.. raw:: html

   <div class="version-selector">
       <label for="version">Version:</label>
       <select id="version" onchange="window.location.href=this.value;">
           <option value="../latest/">Latest</option>
           <option value="../v2.2.0/">v2.2.0</option>
           <option value="../v2.1.4/">v2.1.4</option>
       </select>
   </div>


**Version**: |version|
