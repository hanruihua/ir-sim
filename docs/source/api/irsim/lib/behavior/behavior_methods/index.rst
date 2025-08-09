irsim.lib.behavior.behavior_methods
===================================

.. py:module:: irsim.lib.behavior.behavior_methods


Functions
---------

.. autoapisummary::

   irsim.lib.behavior.behavior_methods.beh_diff_rvo
   irsim.lib.behavior.behavior_methods.beh_diff_dash
   irsim.lib.behavior.behavior_methods.beh_omni_dash
   irsim.lib.behavior.behavior_methods.beh_omni_rvo
   irsim.lib.behavior.behavior_methods.beh_acker_dash
   irsim.lib.behavior.behavior_methods.OmniRVO
   irsim.lib.behavior.behavior_methods.DiffRVO
   irsim.lib.behavior.behavior_methods.OmniDash
   irsim.lib.behavior.behavior_methods.DiffDash
   irsim.lib.behavior.behavior_methods.AckerDash


Module Contents
---------------

.. py:function:: beh_diff_rvo(ego_object: Any, external_objects: list[Any], **kwargs: Any) -> numpy.ndarray

   Behavior function for differential drive robot using RVO (Reciprocal Velocity Obstacles).

   :param ego_object: The ego robot object.
   :param external_objects: List of external objects in the environment.
   :type external_objects: list
   :param \*\*kwargs: Additional keyword arguments:
                      - vxmax (float): Maximum x velocity, default 1.5.
                      - vymax (float): Maximum y velocity, default 1.5.
                      - acce (float): Acceleration factor, default 1.0.
                      - factor (float): Additional scaling factor, default 1.0.
                      - mode (str): RVO calculation mode, default "rvo".
                      - neighbor_threshold (float): Neighbor threshold distance, default 10.0.

   :returns: Velocity [linear, angular] (2x1) for differential drive.
   :rtype: np.array


.. py:function:: beh_diff_dash(ego_object: Any, external_objects: list[Any], **kwargs: Any) -> numpy.ndarray

   Behavior function for differential drive robot using dash-to-goal behavior.

   :param ego_object: The ego robot object.
   :param external_objects: List of external objects in the environment.
   :type external_objects: list
   :param \*\*kwargs: Additional keyword arguments:
                      - angle_tolerance (float): Allowable angular deviation, default 0.1.

   :returns: Velocity [linear, angular] (2x1) for differential drive.
   :rtype: np.array


.. py:function:: beh_omni_dash(ego_object: Any, external_objects: list[Any], **kwargs: Any) -> numpy.ndarray

   Behavior function for omnidirectional robot using dash-to-goal behavior.

   :param ego_object: The ego robot object.
   :param external_objects: List of external objects in the environment.
   :type external_objects: list
   :param \*\*kwargs: Additional keyword arguments (currently unused).

   :returns: Velocity [vx, vy] (2x1) for omnidirectional drive.
   :rtype: np.array


.. py:function:: beh_omni_rvo(ego_object: Any, external_objects: list[Any], **kwargs: Any) -> numpy.ndarray

   Behavior function for omnidirectional robot using RVO (Reciprocal Velocity Obstacles).

   :param ego_object: The ego robot object.
   :param external_objects: List of external objects in the environment.
   :type external_objects: list
   :param \*\*kwargs: Additional keyword arguments:
                      - vxmax (float): Maximum x velocity, default 1.5.
                      - vymax (float): Maximum y velocity, default 1.5.
                      - acce (float): Acceleration factor, default 1.0.
                      - factor (float): Additional scaling factor, default 1.0.
                      - mode (str): RVO calculation mode, default "rvo".
                      - neighbor_threshold (float): Neighbor threshold distance, default 3.0.

   :returns: Velocity [vx, vy] (2x1) for omnidirectional drive.
   :rtype: np.array


.. py:function:: beh_acker_dash(ego_object: Any, external_objects: list[Any], **kwargs: Any) -> numpy.ndarray

   Behavior function for Ackermann steering robot using dash-to-goal behavior.

   :param ego_object: The ego robot object.
   :param external_objects: List of external objects in the environment.
   :type external_objects: list
   :param \*\*kwargs: Additional keyword arguments:
                      - angle_tolerance (float): Allowable angular deviation, default 0.1.

   :returns: Velocity [linear, steering angle] (2x1) for Ackermann drive.
   :rtype: np.array


.. py:function:: OmniRVO(state_tuple: Any, neighbor_list: Optional[list[Any]] = None, vxmax: float = 1.5, vymax: float = 1.5, acce: float = 1, factor: float = 1.0, mode: str = 'rvo', neighbor_threshold: float = 3.0) -> numpy.ndarray

   Calculate the omnidirectional velocity using RVO.

   :param state_tuple: Current state and orientation.
   :type state_tuple: tuple
   :param neighbor_list: List of neighboring agents (default None).
   :type neighbor_list: list
   :param vxmax: Maximum x velocity (default 1.5).
   :type vxmax: float
   :param vymax: Maximum y velocity (default 1.5).
   :type vymax: float
   :param acce: Acceleration factor (default 1).
   :type acce: float
   :param factor: Additional scaling factor (default 1.0).
   :type factor: float
   :param mode: RVO calculation mode (default "rvo").
   :type mode: str
   :param neighbor_threshold: Neighbor threshold (default 3.0).
   :type neighbor_threshold: float

   :returns: Velocity [vx, vy] (2x1).
   :rtype: np.array


.. py:function:: DiffRVO(state_tuple: Any, neighbor_list: Optional[list[Any]] = None, vxmax: float = 1.5, vymax: float = 1.5, acce: float = 1, factor: float = 1.0, mode: str = 'rvo', neighbor_threshold: float = 3.0) -> numpy.ndarray

   Calculate the differential drive velocity using RVO.

   :param state_tuple: Current state and orientation.
   :type state_tuple: tuple
   :param neighbor_list: List of neighboring agents (default None).
   :type neighbor_list: list
   :param vxmax: Maximum x velocity (default 1.5).
   :type vxmax: float
   :param vymax: Maximum y velocity (default 1.5).
   :type vymax: float
   :param acce: Acceleration factor (default 1).
   :type acce: float
   :param factor: Additional scaling factor (default 1.0).
   :type factor: float
   :param mode: RVO calculation mode (default "rvo").
   :type mode: str
   :param neighbor_threshold: Neighbor threshold (default 3.0).
   :type neighbor_threshold: float

   :returns: Velocity [linear, angular] (2x1).
   :rtype: np.array


.. py:function:: OmniDash(state: numpy.ndarray, goal: numpy.ndarray, max_vel: numpy.ndarray, goal_threshold: float = 0.1) -> numpy.ndarray

   Calculate the omnidirectional velocity to reach a goal.

   :param state: Current state [x, y] (2x1).
   :type state: np.array
   :param goal: Goal position [x, y] (2x1).
   :type goal: np.array
   :param max_vel: Maximum velocity [vx, vy] (2x1).
   :type max_vel: np.array
   :param goal_threshold: Distance threshold to consider goal reached (default 0.1).
   :type goal_threshold: float

   :returns: Velocity [vx, vy] (2x1).
   :rtype: np.array


.. py:function:: DiffDash(state: numpy.ndarray, goal: numpy.ndarray, max_vel: numpy.ndarray, goal_threshold: float = 0.1, angle_tolerance: float = 0.2) -> numpy.ndarray

   Calculate the differential drive velocity to reach a goal.

   :param state: Current state [x, y, theta] (3x1).
   :type state: np.array
   :param goal: Goal position [x, y] (2x1).
   :type goal: np.array
   :param max_vel: Maximum velocity [linear, angular] (2x1).
   :type max_vel: np.array
   :param goal_threshold: Distance threshold to consider goal reached (default 0.1).
   :type goal_threshold: float
   :param angle_tolerance: Allowable angular deviation (default 0.2).
   :type angle_tolerance: float

   :returns: Velocity [linear, angular] (2x1).
   :rtype: np.array


.. py:function:: AckerDash(state: numpy.ndarray, goal: numpy.ndarray, max_vel: numpy.ndarray, goal_threshold: float, angle_tolerance: float) -> numpy.ndarray

   Calculate the Ackermann steering velocity to reach a goal.

   :param state: Current state [x, y, theta] (3x1).
   :type state: np.array
   :param goal: Goal position [x, y] (2x1).
   :type goal: np.array
   :param max_vel: Maximum velocity [linear, steering angle] (2x1).
   :type max_vel: np.array
   :param goal_threshold: Distance threshold to consider goal reached.
   :type goal_threshold: float
   :param angle_tolerance: Allowable angular deviation.
   :type angle_tolerance: float

   :returns: Velocity [linear, steering angle] (2x1).
   :rtype: np.array


