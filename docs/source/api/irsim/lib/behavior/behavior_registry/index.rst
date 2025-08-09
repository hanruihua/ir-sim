irsim.lib.behavior.behavior_registry
====================================

.. py:module:: irsim.lib.behavior.behavior_registry


Attributes
----------

.. autoapisummary::

   irsim.lib.behavior.behavior_registry.behaviors_map


Functions
---------

.. autoapisummary::

   irsim.lib.behavior.behavior_registry.register_behavior


Module Contents
---------------

.. py:data:: behaviors_map
   :type:  dict[tuple[str, str], Callable[Ellipsis, Any]]

.. py:function:: register_behavior(kinematics: str, action: str)

   decorator to register a method in the behavior registry

   :param kinematics: only support diff, omni, or acker
   :param action: defined action for the kinematics


