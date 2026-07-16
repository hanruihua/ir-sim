API Reference
=============

Welcome to the IR-SIM API Reference. This section provides detailed documentation for all modules, classes, and functions in the IR-SIM package.

Core simulation loop
--------------------

The handful of entry points you will use most; everything else is reachable from the modules below.

- :py:func:`irsim.make`: create an environment from a YAML scenario.
- :py:meth:`~irsim.env.env_base.EnvBase.step`: advance the simulation by one step.
- :py:meth:`~irsim.env.env_base.EnvBase.render`: draw the current state.
- :py:meth:`~irsim.env.env_base.EnvBase.done`: check whether a terminal condition is reached.
- :py:meth:`~irsim.env.env_base.EnvBase.reset`: restore objects to their initial states.
- :py:meth:`~irsim.env.env_base.EnvBase.end`: close the environment and release resources.
- :py:meth:`~irsim.env.env_base.EnvBase.get_robot_state`: read the primary robot's state.
- :py:meth:`~irsim.env.env_base.EnvBase.get_lidar_scan`: read a robot's LiDAR scan.

How to read this reference
--------------------------

The API pages below are generated from the Python package with AutoAPI. Start
from the high-level entry points when writing simulations, and use the lower
level modules when extending IR-SIM itself.

.. list-table::
   :header-rows: 1

   * - Task
     - Main API
     - Notes
   * - Create and run environments
     - :py:func:`irsim.make`, :py:class:`~irsim.env.env_base.EnvBase`
     - Preferred public entry points for most users.
   * - Inspect or control objects
     - :py:class:`~irsim.world.object_base.ObjectBase`
     - Common base for robots, obstacles, and map objects.
   * - Create objects from YAML
     - :py:class:`~irsim.world.object_factory.ObjectFactory`
     - Used internally by the YAML loader; useful for advanced programmatic setup.
   * - Add kinematics models
     - :py:func:`~irsim.lib.handler.kinematics_handler.register_kinematics`,
       :py:class:`~irsim.lib.handler.kinematics_handler.KinematicsHandler`
     - Register models under the same names used by YAML ``kinematics``.
   * - Add object geometry
     - :py:class:`~irsim.lib.handler.geometry_handler.GeometryFactory`
     - Converts YAML ``shape`` dictionaries into Shapely geometries.
   * - Add sensors
     - :py:class:`~irsim.world.sensors.sensor_factory.SensorFactory`
     - Creates concrete sensors from YAML ``sensors`` entries.
   * - Use path planners
     - :mod:`irsim.lib.path_planners`
     - Grid and sampling planners operate on :class:`~irsim.world.map.EnvGridMap`.

Generated pages may include inherited members so that re-exported public APIs
are still discoverable. When defaults differ between YAML configuration and a
lower-level constructor, the YAML reference documents the YAML behavior, while
the API reference documents the direct Python constructor behavior.

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
