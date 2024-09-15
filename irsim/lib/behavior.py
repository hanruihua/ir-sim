from math import inf
import numpy as np
from irsim.global_param import world_param
from irsim.util.util import relative_position, WrapToPi
from irsim.lib.behaviorlib import DiffDash, AckerDash, DiffRVO, OmniDash, OmniRVO


class Behavior:
    """

    A class to represent the behavior of an agent in the simulation.

    Args:
        object_info (object): Object Information from the object_base class ObjectInfo.
        behavior_dict (dict) : Dictionary containing behavior parameters for different behavior.
            name: dash, wander, rvo
    """

    def __init__(self, object_info=None, behavior_dict=None) -> None:

        self.object_info = object_info
        self.behavior_dict = behavior_dict

    def gen_vel(self, state, goal, min_vel, max_vel, **kwargs):
        """
        Generate velocity for the agent based on the behavior dictionary.

        Args:
            state (np.array): Current state of the agent. x, y, theta.
            goal (np.array): Goal state of the agent. x, y, theta.
            min_vel (np.array): Minimum velocity of the agent. (2, 1) control vector
            max_vel (np.array): Maximum velocity of the agent. (2, 1) control vector
            **kwargs : Additional arguments for the behavior.

        """

        if self.behavior_dict is None:
            return np.zeros((2, 1))

        if self.object_info.kinematics == "diff":
            if self.behavior_dict["name"] == "dash":

                angle_tolerance = self.behavior_dict.get("angle_tolerance", 0.1)
                goal_threshold = self.object_info.goal_threshold

                behavior_vel = DiffDash(
                    state, goal, max_vel, angle_tolerance, goal_threshold
                )

            elif self.behavior_dict["name"] == "wander":

                angle_tolerance = self.behavior_dict.get("angle_tolerance", 0.1)
                goal_threshold = self.object_info.goal_threshold

                behavior_vel = DiffDash(
                    state, goal, max_vel, angle_tolerance, goal_threshold
                )

            elif self.behavior_dict["name"] == "rvo":

                # state_tuple, neighbor_list=None, vxmax = 1.5, vymax = 1.5, acceler = 0.5, mode='rvo'
                # state_tuple, neighbor_list=None,
                rvo_neighbor = kwargs.get("rvo_neighbor", [])
                rvo_state = kwargs.get("rvo_state", [])

                vxmax = self.behavior_dict.get("vxmax", 1.5)
                vymax = self.behavior_dict.get("vymax", 1.5)
                acceler = self.behavior_dict.get("acceler", 1.0)
                factor = self.behavior_dict.get("factor", 1.0)
                mode = self.behavior_dict.get("mode", "rvo")

                behavior_vel = DiffRVO(
                    rvo_state, rvo_neighbor, vxmax, vymax, acceler, factor, mode
                )

        elif self.object_info.kinematics == "acker":

            if self.behavior_dict["name"] == "dash":

                angle_tolerance = self.behavior_dict.get("angle_tolerance", 0.1)
                goal_threshold = self.object_info.goal_threshold

                behavior_vel = AckerDash(
                    state, goal, max_vel, angle_tolerance, goal_threshold
                )

        elif self.object_info.kinematics == "omni":

            if self.behavior_dict["name"] == "dash":
                goal_threshold = self.object_info.goal_threshold
                behavior_vel = OmniDash(state, goal, max_vel, goal_threshold)

            elif self.behavior_dict["name"] == "rvo":

                rvo_neighbor = kwargs.get("rvo_neighbor", [])
                rvo_state = kwargs.get("rvo_state", [])

                vxmax = self.behavior_dict.get("vxmax", 1.5)
                vymax = self.behavior_dict.get("vymax", 1.5)
                acceler = self.behavior_dict.get("acceler", 1.0)
                factor = self.behavior_dict.get("factor", 1.0)
                mode = self.behavior_dict.get("mode", "rvo")

                behavior_vel = OmniRVO(
                    rvo_state, rvo_neighbor, vxmax, vymax, acceler, factor, mode
                )

        return behavior_vel
