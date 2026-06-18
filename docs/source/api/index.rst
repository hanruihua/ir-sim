API Reference
=============

Welcome to the IR-SIM API Reference. This section provides detailed documentation for all modules, classes, and functions in the IR-SIM package.

Core simulation loop
--------------------

The handful of entry points you will use most — everything else is reachable from the modules below.

- :py:func:`irsim.make` — create an environment from a YAML scenario.
- :py:meth:`~irsim.env.env_base.EnvBase.step` — advance the simulation by one step.
- :py:meth:`~irsim.env.env_base.EnvBase.render` — draw the current state.
- :py:meth:`~irsim.env.env_base.EnvBase.done` — check whether a terminal condition is reached.
- :py:meth:`~irsim.env.env_base.EnvBase.reset` — restore objects to their initial states.
- :py:meth:`~irsim.env.env_base.EnvBase.end` — close the environment and release resources.
- :py:meth:`~irsim.env.env_base.EnvBase.get_robot_state` — read the primary robot's state.
- :py:meth:`~irsim.env.env_base.EnvBase.get_lidar_scan` — read a robot's LiDAR scan.

.. toctree::
   :maxdepth: 1
   :caption: API Documentation

   Overview (irsim.make, EnvBase) <irsim/index>
   irsim/env/index
   irsim/world/index
   irsim/lib/index
   irsim/gui/index
   irsim/util/index
   irsim/config/index