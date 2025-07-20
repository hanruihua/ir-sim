import numpy as np
from irsim.config import env_param, world_param
import importlib
from typing import Tuple, Any
from irsim.lib.behavior.behavior_registry import behaviors_map



class Behavior:
    """
    Represents the behavior of an agent in the simulation.

    Args:
        object_info (object): Object information from the object_base class ObjectInfo.
        behavior_dict (dict): Dictionary containing behavior parameters for different behaviors.
            Name Options include: 'dash', 'rvo'.
            target_roles:
            
            - 'all': all objects in the environment will be considered within this behavior.
            - 'obstacle': only obstacles will be considered within this behavior.
            - 'robot': only robots will be considered within this behavior.
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

    def gen_vel(self, ego_object, external_objects=[]):
        """
        Generate velocity for the agent based on the behavior dictionary.

        Args:
            ego_object: the object itself
            external_objects: all the other objects in the environment

        Returns:
            np.array (2, 1): Generated velocity for the agent.
        """

        if self.behavior_dict is None or not self.behavior_dict:
            
            if world_param.control_mode == "auto":
                if world_param.count == 1:
                    self.logger.warning(
                        "Behavior not defined for Object {}. This object will be static. Available behaviors: rvo, dash".format(
                            self.object_info.id,
                        )
                    )
                    
            return np.zeros((2, 1))

        target_roles = self.behavior_dict.get("target_roles", "all")

        if target_roles == "all":
            external_objects = external_objects
        elif target_roles == "obstacle":
            external_objects = [
                obj for obj in external_objects if obj.role == "obstacle"
            ]
        elif target_roles == "robot":
            external_objects = [obj for obj in external_objects if obj.role == "robot"]

        behavior_vel = self.invoke_behavior(
            self.object_info.kinematics,
            self.behavior_dict["name"],
            ego_object=ego_object,
            external_objects=external_objects,
            **self.behavior_dict,
        )

        return behavior_vel

    def load_behavior(self, behaviors: str = ".behavior_methods"):
        """
        Load behavior parameters from the script.

        Args:
            behaviors (str): name of the bevavior script.
        """

        try:
            importlib.import_module(behaviors, package="irsim.lib.behavior")
        except ImportError as e:
            print(f"Failed to load module '{behaviors}': {e}")

    def invoke_behavior(self, kinematics: str, action: str, **kwargs: Any) -> Any:
        """
        Invoke a specific behavior method based on kinematics model and action type.

        This method looks up and executes the appropriate behavior function from the
        behavior registry based on the combination of kinematics model and action name.

        Args:
            kinematics (str): Kinematics model identifier. Supported values:
                
                - 'diff': Differential drive kinematics
                - 'omni': Omnidirectional kinematics  
                - 'acker': Ackermann steering kinematics
                
            action (str): Behavior action name. Examples:
                
                - 'dash': Direct movement toward goal
                - 'rvo': Reciprocal Velocity Obstacles for collision avoidance
                
            **kwargs: Additional keyword arguments passed to the behavior function.
                Common parameters include ego_object, external_objects, goal, etc.

        Returns:
            np.ndarray: Generated velocity vector (2x1) in the format appropriate
            for the specified kinematics model.

        Raises:
            ValueError: If no behavior method is found for the given kinematics
                and action combination.

        Example:
            >>> # Invoke differential drive dash behavior
            >>> vel = behavior.invoke_behavior('diff', 'dash', 
            ...                               ego_object=robot, 
            ...                               external_objects=obstacles)
        """
        key: Tuple[str, str] = (kinematics, action)
        func = behaviors_map.get(key)
        if not func:
            raise ValueError(
                f"No method found for category '{kinematics}' and action '{action}'."
            )

        return func(**kwargs)
    
    @property
    def logger(self):
        return env_param.logger
