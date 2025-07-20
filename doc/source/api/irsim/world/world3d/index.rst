irsim.world.world3d
===================

.. py:module:: irsim.world.world3d


Classes
-------

.. autoapisummary::

   irsim.world.world3d.World3D


Module Contents
---------------

.. py:class:: World3D(name, depth: float = 10.0, offset=[0, 0, 0], **kwargs)

   Bases: :py:obj:`irsim.world.World`


   Represents the main simulation environment, managing objects and maps.

   .. attribute:: name

      Name of the world.

      :type: str

   .. attribute:: height

      Height of the world.

      :type: float

   .. attribute:: width

      Width of the world.

      :type: float

   .. attribute:: step_time

      Time interval between steps.

      :type: float

   .. attribute:: sample_time

      Time interval between samples.

      :type: float

   .. attribute:: offset

      Offset for the world's position.

      :type: list

   .. attribute:: control_mode

      Control mode ('auto' or 'keyboard').

      :type: str

   .. attribute:: collision_mode

      Collision mode ('stop',  , 'unobstructed').

      :type: str

   .. attribute:: obstacle_map

      Image file for the obstacle map.

   .. attribute:: mdownsample

      Downsampling factor for the obstacle map.

      :type: int

   .. attribute:: status

      Status of the world and objects.

   .. attribute:: plot

      Plot configuration for the world.

   Initialize the world object.

   :param name: Name of the world.
   :type name: str
   :param height: Height of the world.
   :type height: float
   :param width: Width of the world.
   :type width: float
   :param step_time: Time interval between steps.
   :type step_time: float
   :param sample_time: Time interval between samples.
   :type sample_time: float
   :param offset: Offset for the world's position.
   :type offset: list
   :param control_mode: Control mode ('auto' or 'keyboard').
   :type control_mode: str
   :param collision_mode: Collision mode ('stop',  , 'unobstructed').
   :type collision_mode: str
   :param obstacle_map: Image file for the obstacle map.
   :param mdownsample: Downsampling factor for the obstacle map.
   :type mdownsample: int
   :param plot: Plot configuration.
   :type plot: dict
   :param status: Initial simulation status.
   :type status: str


   .. py:attribute:: depth
      :value: 10.0



   .. py:attribute:: offset
      :value: [0, 0, 0]



   .. py:attribute:: z_range


