irsim.world.world
=================

.. py:module:: irsim.world.world


Classes
-------

.. autoapisummary::

   irsim.world.world.World


Module Contents
---------------

.. py:class:: World(name: Optional[str] = 'world', height: float = 10, width: float = 10, step_time: float = 0.1, sample_time: float = 0.1, offset: list = [0, 0], control_mode: str = 'auto', collision_mode: str = 'stop', obstacle_map=None, mdownsample: int = 1, plot: dict = dict(), status: str = 'None', **kwargs)

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


   .. py:attribute:: name


   .. py:attribute:: height
      :value: 10



   .. py:attribute:: width
      :value: 10



   .. py:attribute:: step_time
      :value: 0.1



   .. py:attribute:: sample_time
      :value: 0.1



   .. py:attribute:: offset
      :value: [0, 0]



   .. py:attribute:: count
      :value: 0



   .. py:attribute:: sampling
      :value: True



   .. py:attribute:: x_range


   .. py:attribute:: y_range


   .. py:attribute:: plot_parse


   .. py:attribute:: status
      :value: 'None'



   .. py:method:: step()

      Advance the simulation by one step.



   .. py:method:: gen_grid_map(obstacle_map, mdownsample=1)

      Generate a grid map for obstacles.

      :param obstacle_map: Path to the obstacle map image.
      :param mdownsample: Downsampling factor.
      :type mdownsample: int

      :returns: Grid map, obstacle indices, and positions.
      :rtype: tuple



   .. py:method:: get_map(resolution: float = 0.1, obstacle_list: list = [])

      Get the map of the world with the given resolution.



   .. py:method:: reset()

      Reset the world simulation.



   .. py:property:: time

      Get the current simulation time.

      :returns: Current time based on steps and step_time.
      :rtype: float


   .. py:property:: buffer_reso

      Get the maximum resolution of the world.

      :returns: Maximum resolution.
      :rtype: float


   .. py:method:: rgb2gray(rgb)


