from math import inf
import numpy as np
from ir_sim.global_param import world_param
from ir_sim.util.util import relative_position, WrapToPi
from ir_sim.lib.behaviorlib import DiffDash, AckerDash, DiffRVO


class Behavior:
    def __init__(self, object_info=None, behavior_dict=None) -> None:

        self.object_info = object_info
        self.behavior_dict = behavior_dict


    def gen_vel(self, state, goal, min_vel, max_vel, **kwargs):

        if self.behavior_dict is None:
            return np.zeros((2, 1))

        if self.object_info.kinematics == 'diff':
            if self.behavior_dict['name'] == 'dash':

                angle_tolerance = self.behavior_dict.get('angle_tolerance', 0.1)
                goal_threshold = self.object_info.goal_threshold

                behavior_vel = DiffDash(state, goal, max_vel, angle_tolerance, goal_threshold)

            elif self.behavior_dict['name'] == 'wander':
                
                angle_tolerance = self.behavior_dict.get('angle_tolerance', 0.1)
                goal_threshold = self.object_info.goal_threshold

                behavior_vel = DiffDash(state, goal, max_vel, angle_tolerance, goal_threshold)

            elif self.behavior_dict['name'] == 'rvo':

                # state_tuple, neighbor_list=None, vxmax = 1.5, vymax = 1.5, acceler = 0.5, mode='rvo'
                # state_tuple, neighbor_list=None, 
                rvo_neighbor = kwargs.get('rvo_neighbor', [])
                rvo_state = kwargs.get('rvo_state', [])


                vxmax = self.behavior_dict.get('vxmax', 1.5)
                vymax = self.behavior_dict.get('vymax', 1.5)
                acceler = self.behavior_dict.get('acceler', 1.0)
                factor = self.behavior_dict.get('factor', 1.0)
                mode = self.behavior_dict.get('mode', 'rvo')

                behavior_vel = DiffRVO(rvo_state, rvo_neighbor, vxmax, vymax, acceler, factor, mode)
            
        elif self.object_info.kinematics == 'acker':

            if self.behavior_dict['name'] == 'dash':

                angle_tolerance = self.behavior_dict.get('angle_tolerance', 0.1)
                goal_threshold = self.object_info.goal_threshold

                behavior_vel = AckerDash(state, goal, max_vel, angle_tolerance, goal_threshold)

        elif self.object_info.kinematics == 'omni':
            pass
            
                
        return behavior_vel

    






        
        