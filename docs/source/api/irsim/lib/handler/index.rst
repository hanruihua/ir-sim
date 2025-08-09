irsim.lib.handler
=================

.. py:module:: irsim.lib.handler

.. autoapi-nested-parse::

   Handler classes for IR-SIM simulation.

   This package contains factory classes for:
   - kinematics_handler: Kinematics factory
   - geometry_handler: Geometry factory



Submodules
----------

.. toctree::
   :maxdepth: 1

   /api/irsim/lib/handler/geometry_handler/index
   /api/irsim/lib/handler/kinematics_handler/index


Classes
-------

.. autoapisummary::

   irsim.lib.handler.GeometryFactory
   irsim.lib.handler.KinematicsFactory


Package Contents
----------------

.. py:class:: GeometryFactory

   Factory class to create geometry handlers.


   .. py:method:: create_geometry(name: str = 'circle', **kwargs) -> geometry_handler
      :staticmethod:



.. py:class:: KinematicsFactory

   Factory class to create kinematics handlers.


   .. py:method:: create_kinematics(name: Optional[str] = None, noise: bool = False, alpha: Optional[list] = None, mode: str = 'steer', wheelbase: Optional[float] = None, role: str = 'robot') -> KinematicsHandler
      :staticmethod:



