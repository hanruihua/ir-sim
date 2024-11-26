from math import inf
import numpy as np
from irsim.global_param import env_param
import importlib
from typing import Tuple, Any
from .behavior_registry import behaviors_map

class Behavior:
    """
    Represents the behavior of an agent in the simulation.

    Args:
        object_info (object): Object information from the object_base class ObjectInfo.
        behavior_dict (dict): Dictionary containing behavior parameters for different behaviors.
            Options include: 'dash', 'rvo'.
    """

    def __init__(self, object_info=None, behavior_dict=None) -> None:
        """
        Initializes the Behavior class with object information and behavior parameters.

        Args:
            object_info (object): Information about the agent.
            behavior_dict (dict): Behavior parameters.
        """
        self.object_info = object_info
        self.behavior_dict = dict() if behavior_dict is None else behavior_dict
        self.load_behavior()

    def gen_vel(self, objects):
        """
        Generate velocity for the agent based on the behavior dictionary.

        Args:
            objects: all the objects in the evironment

        Returns:
            np.array (2, 1): Generated velocity for the agent. 
        """
        
        if self.behavior_dict is None:
            env_param.logger.error("Behavior not defined for object {}.".format(self.object_info.id))
            return np.zeros((2, 1))

        behavior_vel = self.invoke_behavior(self.object_info.kinematics, self.behavior_dict["name"], ego_object=objects[self.object_info.id], objects=objects, **self.behavior_dict)

        return behavior_vel


    def load_behavior(self, behaviors: str='.behavior_methods'):
        """
        Load behavior parameters from the script.

        Args:
            behaviors (str): name of the bevavior script.
        """

        try:
            importlib.import_module(behaviors, package='irsim.lib.behavior')
        except ImportError as e:
            print(f"Failed to load module '{behaviors}': {e}")


    def invoke_behavior(self, kinematics: str, action: str, **kwargs: Any) -> Any:
        """
        Invoke a behavior method.

        Args:
            kinematics (str): Name of the behavior method. only support: 'diff', 'omni', 'acker'.
            action: Name of the action method. example: 'dash', 'rvo'.
            **kwargs: Arbitrary keyword arguments.

        Returns:
            np.array: Velocity (2x1).
        """
        key: Tuple[str, str] = (kinematics, action)
        func = behaviors_map.get(key)
        if not func:
            raise ValueError(f"No method found for category '{kinematics}' and action '{action}'.")
        
        return func(**kwargs)
