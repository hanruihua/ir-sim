from operator import length_hint
import numpy as np
from ir_sim2.util.util import random_points
from math import pi

class EnvObstacle:

    def __init__(self, obs_class, number=1, distribute='manual', dynamic=False, step_time=0.1, sport='default', **kwargs):

        self.obs_class = obs_class
        self.number = number
        self.distribute = distribute
        self.dynamic = dynamic
        self.step_time = step_time
        self.sport = sport # default, wander, patrol
        self.obs_list = []

        if obs_class.obstacle_type == 'obstacle_circle':
            
            self.obs_cir_list = []
            
            if distribute == 'manual':
                center_list = kwargs.get('center_list', None)
                goal_list = kwargs.get('goal_list', None)
                radius_list = kwargs.get('radius_list', [0.2] * number)
                
                if isinstance(radius_list, float): radius_list = [radius_list] * number
                if len(radius_list) < number: radius_list.extend([radius_list[-1]]* (number - len(radius_list)) )

            elif distribute == 'random':

                range_low = kwargs.get('range_low', [0, 0, 0])
                range_high = kwargs.get('range_high', [10, 10])
                center_distance = kwargs.get('center_distance', 1)
                
                center_list = random_points(number, np.c_[range_low], np.c_[range_high], center_distance)
                goal_list = random_points(number, np.c_[range_low], np.c_[range_high], center_distance)

                if kwargs.get('random_radius', False):
                    radius_list = np.random.uniform(low=kwargs.get('radius_low', 0.2), high = kwargs.get('radius_high', 1), size = (number,))
                else:
                    radius_list = kwargs.get('radius_list', [0.2] * number)
                
                if len(radius_list) < number: radius_list.extend([radius_list[-1]]* (number - len(radius_list)) )

            if number > 0:
                for id, radius, center, goal in zip(range(number), radius_list, center_list, goal_list):
                    obstacle = obs_class(id=id, center=center, goal=goal, radius=radius, step_time=self.step_time, **kwargs)
                    self.obs_cir_list.append(obstacle)
                    self.obs_list.append(obstacle)

        elif obs_class.obstacle_type == 'obstacle_polygon':
            self.obs_poly_list = []

            if distribute == 'manual':
                vertex_list = kwargs.get('vertex_list', None)
                state_list = kwargs.get('state_list', np.zeros((3, 1)))
                
            if number > 0:

                if len(state_list) < number: state_list.extend([state_list[-1]]* (number - len(state_list)) )
                if len(vertex_list) < number: vertex_list.extend([vertex_list[-1]]* (number - len(vertex_list)) )

                for id, state, vertex in zip(range(number), state_list, vertex_list):
                    obstacle = obs_class(id=id, state=state, vertex=vertex, **kwargs) 
                    self.obs_poly_list.append(obstacle)
                    self.obs_list.append(obstacle)

        elif obs_class.obstacle_type == 'obstacle_block':
            self.obs_block_list = []

            length_list = kwargs.get('length_list', None)
            width_list = kwargs.get('width_list', None)

            if len(length_list) < number:
                ext_list = [length_list[-1]] * (number - len(length_list))
                length_list.extend(ext_list)
            
            if len(width_list) < number:
                ext_list = [width_list[-1]] * (number - len(width_list))
                width_list.extend(ext_list)

            if distribute == 'manual':
                center_list = kwargs.get('center_list', None)
                
            elif distribute == 'random':
                # number, low, high, center_distance, max_iter=100
                range_low = kwargs.get('range_low', [0, 0, 0])
                range_high = kwargs.get('range_high', [10, 10, 2*pi])
                center_distance = kwargs.get('center_distance', 1)

                center_list = random_points(number, np.c_[range_low], np.c_[range_high], center_distance)

            if number > 0:
                for id, length, width, center in zip(range(number), length_list, width_list, center_list):
                    obstacle = obs_class(id=id, center=center, length=length, width=width, step_time=self.step_time, **kwargs)
                    self.obs_block_list.append(obstacle)
                    self.obs_list.append(obstacle)        

        elif obs_class.obstacle_type == 'obstacle_map':
            pass

        
    def move(self, **kwargs):

        if self.dynamic:
            if self.sport == 'default':
                [obs.move_goal(**kwargs) for obs in self.obs_cir_list]
            elif self.sport == 'wander':
                [obs.move_wander(**kwargs) for obs in self.obs_cir_list]
            
    def collision_check(self):
        pass
    
    def collision_check(self, robot, obstacle):
        pass
    
    def collision_check_point(self, point):

        for obs in self.obs_list:
            if obs.collision_check_point(point): return True
        
        return False

    def reset(self):
        if self.dynamic:
            [obs.reset() for obs in self.obs_list]

    def plot(self, ax, **kwargs):
        [obs.plot(ax, **kwargs) for obs in self.obs_list]
 
    def plot_clear(self):
        [obs.plot_clear() for obs in self.obs_list]


