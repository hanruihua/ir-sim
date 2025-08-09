irsim.lib.behavior.behavior
===========================

.. py:module:: irsim.lib.behavior.behavior


Classes
-------

.. autoapisummary::

   irsim.lib.behavior.behavior.Behavior


Module Contents
---------------

.. py:class:: Behavior(object_info=None, behavior_dict=None)

   Represents the behavior of an agent in the simulation.

   :param object_info: Object information from the object_base class ObjectInfo.
   :type object_info: object
   :param behavior_dict: Dictionary containing behavior parameters for different behaviors.
                         Name Options include: 'dash', 'rvo'.
                         target_roles:

                         - 'all': all objects in the environment will be considered within this behavior.
                         - 'obstacle': only obstacles will be considered within this behavior.
                         - 'robot': only robots will be considered within this behavior.
   :type behavior_dict: dict

   Initializes the Behavior class with object information and behavior parameters.

   :param object_info: Information about the agent.
   :type object_info: object
   :param behavior_dict: Behavior parameters.
   :type behavior_dict: dict


   .. py:attribute:: object_info
      :value: None



   .. py:attribute:: behavior_dict


   .. py:method:: gen_vel(ego_object, external_objects=None)

      Generate velocity for the agent based on the behavior dictionary.

      :param ego_object: the object itself
      :param external_objects: all the other objects in the environment

      :returns: Generated velocity for the agent.
      :rtype: np.array (2, 1)



   .. py:method:: load_behavior(behaviors: str = '.behavior_methods')

      Load behavior parameters from the script.

      :param behaviors: name of the bevavior script.
      :type behaviors: str



   .. py:method:: invoke_behavior(kinematics: str, action: str, **kwargs: Any) -> Any

      Invoke a specific behavior method based on kinematics model and action type.

      This method looks up and executes the appropriate behavior function from the
      behavior registry based on the combination of kinematics model and action name.

      :param kinematics: Kinematics model identifier. Supported values:

                         - 'diff': Differential drive kinematics
                         - 'omni': Omnidirectional kinematics
                         - 'acker': Ackermann steering kinematics
      :type kinematics: str
      :param action: Behavior action name. Examples:

                     - 'dash': Direct movement toward goal
                     - 'rvo': Reciprocal Velocity Obstacles for collision avoidance
      :type action: str
      :param \*\*kwargs: Additional keyword arguments passed to the behavior function.
                         Common parameters include ego_object, external_objects, goal, etc.

      :returns: Generated velocity vector (2x1) in the format appropriate
                for the specified kinematics model.
      :rtype: np.ndarray

      :raises ValueError: If no behavior method is found for the given kinematics
          and action combination.

      .. rubric:: Example

      >>> # Invoke differential drive dash behavior
      >>> vel = behavior.invoke_behavior('diff', 'dash',
      ...                               ego_object=robot,
      ...                               external_objects=obstacles)



   .. py:property:: logger


