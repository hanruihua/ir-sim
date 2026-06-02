.. IR-SIM documentation master file, created by
   sphinx-quickstart on Tue Aug  6 00:59:39 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

:html_theme.sidebar_secondary.remove: true

==================================
Welcome to IR-SIM's documentation!
==================================

**IR-SIM** is an open-source, lightweight Python robot simulator for navigation, control, and learning. It pairs a simple, user-friendly framework with built-in collision detection for modeling robots, sensors, and environments — so you can prototype robotics and AI algorithms in custom scenarios with minimal code and hardware.

.. grid:: 2 3 5 5
    :gutter: 2
    :class-container: hero-cta

    .. grid-item-card:: 🚀 Get Started
        :link: get_started/index
        :link-type: doc
        :text-align: center
        :class-card: hero-cta-primary

    .. grid-item-card:: 📚 User Guide
        :link: usage/index
        :link-type: doc
        :text-align: center

    .. grid-item-card:: ⚙️ Configuration
        :link: yaml_config/index
        :link-type: doc
        :text-align: center

    .. grid-item-card:: 🔧 API Reference
        :link: api/index
        :link-type: doc
        :text-align: center

    .. grid-item-card:: ⭐ GitHub
        :link: https://github.com/hanruihua/ir-sim
        :text-align: center

See IR-SIM in action
====================

.. grid:: 3 3 3 3
    :gutter: 3

    .. grid-item-card::
        :shadow: lg
        :text-align: center

        .. image:: https://github.com/user-attachments/assets/5930b088-d400-4943-8ded-853c22eae75b
           :width: 70%
           :alt: Multi-robot collision avoidance

        +++
        Multi-robot collision avoidance

    .. grid-item-card::
        :shadow: lg
        :text-align: center

        .. image:: https://github.com/user-attachments/assets/3257abc1-8bed-40d8-9b51-e5d90b06ee06
           :width: 70%
           :alt: Navigation in Grid World

        +++
        Navigation in Grid World

    .. grid-item-card::
        :shadow: lg
        :text-align: center

        .. image:: https://github.com/user-attachments/assets/7aa809c2-3a44-4377-a22d-728b9dbdf8bc
           :width: 70%
           :alt: Sensor visualization

        +++
        Sensor visualization

    .. grid-item-card::
        :shadow: lg
        :text-align: center

        .. image:: https://github.com/user-attachments/assets/1cc8a4a6-2f41-4bc9-bc59-a7faff443223
           :width: 70%
           :alt: Dynamic environment simulation

        +++
        Dynamic environment simulation

    .. grid-item-card::
        :shadow: lg
        :text-align: center

        .. image:: https://github.com/user-attachments/assets/0fac81e7-60c0-46b2-91f0-efe4762bb758
           :width: 70%
           :alt: 3D habitat spaces

        +++
        3D habitat spaces

    .. grid-item-card::
        :shadow: lg
        :text-align: center

        .. image:: usage/gif/mouse.gif
           :width: 70%
           :alt: Mouse control

        +++
        Mouse control

Key Features
============

- Simulate a wide range of robot platforms with diverse kinematics, sensors, and behaviors
- Quickly configure and customize simulation scenarios using straightforward YAML files, with no complex coding required
- Visualize simulation outcomes in real time for immediate feedback and analysis using matplotlib
- Support collision detection and behavior control for each object in the simulation

Installation
============

Install the latest release from PyPI:

.. code-block:: bash

    pip install ir-sim

Prefer conda or uv? Pick your installation method:

.. grid:: 1 3 3 3

    .. grid-item-card:: 📦 pip
        :link: get_started/install
        :link-type: doc
        :text-align: center
        :shadow: md

        :bdg-success:`Recommended`

        ^^^

        Quick installation with pip

    .. grid-item-card:: 🐍 conda
        :link: get_started/install
        :link-type: doc
        :text-align: center
        :shadow: md

        :bdg-info:`Popular`

        ^^^

        Installation in conda environment

    .. grid-item-card:: ⚡ uv
        :link: get_started/install
        :link-type: doc
        :text-align: center
        :shadow: md

        :bdg-warning:`Fast`

        ^^^

        Lightning-fast installation

Projects using IR-SIM
=====================

.. grid:: 1 1 1 1

    .. grid-item-card:: Academic projects
        :shadow: md

        * `rl-rvo-nav (RAL & ICRA2023) <https://github.com/hanruihua/rl_rvo_nav>`_
        * `RDA_planner (RAL & IROS2023) <https://github.com/hanruihua/RDA_planner>`_
        * `NeuPAN (T-RO 2025) <https://github.com/hanruihua/NeuPAN>`_

    .. grid-item-card:: Deep Reinforcement Learning Projects
        :shadow: md

        * `DRL-robot-navigation-IR-SIM <https://github.com/reiniscimurs/DRL-robot-navigation-IR-SIM>`_
        * `AutoNavRL <https://github.com/harshmahesheka/AutoNavRL>`_

.. toctree::
   :maxdepth: 2
   :caption: Documentation
   :hidden:

   Getting Started <get_started/index>
   User Guide <usage/index>
   Configuration <yaml_config/index>
   API Reference <api/index>

.. toctree::
   :maxdepth: 2
   :caption: Development
   :hidden:

   Contributing <contributing>
   Changelog <changelog>

----

Get Started
===============

Ready to start using IR-SIM? Choose your preferred installation method:

.. grid:: 1 3 3 3

    .. grid-item-card:: 📦 pip
        :link: get_started/install
        :link-type: doc
        :text-align: center
        :shadow: md

        :bdg-success:`Recommended`
        
        ^^^
        
        Quick installation with pip

    .. grid-item-card:: 🐍 conda
        :link: get_started/install
        :link-type: doc
        :text-align: center
        :shadow: md

        :bdg-info:`Popular`
        
        ^^^
        
        Installation in conda environment

    .. grid-item-card:: ⚡ uv
        :link: get_started/install
        :link-type: doc
        :text-align: center
        :shadow: md

        :bdg-warning:`Fast`
        
        ^^^
        
        Lightning-fast installation

----

Projects using IR-SIM
========================

.. grid:: 1 1 1 1

    .. grid-item-card:: Academic projects
        :shadow: md

        * `rl-rvo-nav (RAL & ICRA2023) <https://github.com/hanruihua/rl_rvo_nav>`_
        * `RDA_planner (RAL & IROS2023) <https://github.com/hanruihua/RDA_planner>`_
        * `NeuPAN (T-RO 2025) <https://github.com/hanruihua/NeuPAN>`_

    .. grid-item-card:: Deep Reinforcement Learning Projects
        :shadow: md

        * `DRL-robot-navigation-IR-SIM <https://github.com/reiniscimurs/DRL-robot-navigation-IR-SIM>`_
        * `AutoNavRL <https://github.com/harshmahesheka/AutoNavRL>`_
        * `IRSIM-3DGS-Bridge <https://github.com/Wayneyujie/IRSIM-3DGS-Bridge>`_ - A closed-loop bridge from 3D Gaussian Splatting scenes to IR-SIM planning/following and Habitat-GS trajectory playback.
