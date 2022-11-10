from operator import length_hint
import numpy as np
from ir_sim2.util.util import random_points, random_value
from math import pi, sin, cos

class EnvObstacle:

    def __init__(self, obs_class, number=1, distribute={}, dynamic={}, step_time=0.1, **kwargs):

        self.obs_class = obs_class
        self.number = number
        self.distribute = distribute
        self.step_time = step_time
        self.obs_list = []
        self.dynamic = dynamic

        if self.number > 0:

            state_list, shape_list = self.init_distribute(self.number, **distribute)

            if obs_class.obstacle_type == 'obstacle_circle':
                self.obs_cir_list = [] 

                for id, radius, center in zip(range(number), shape_list, state_list):
                    obstacle = obs_class(id=id, center=center, radius=radius, step_time=self.step_time, **kwargs)
                    self.obs_cir_list.append(obstacle)
                    self.obs_list.append(obstacle)

            elif obs_class.obstacle_type == 'obstacle_polygon':
                self.obs_poly_list = []

                for id, state, vertex in zip(range(number), state_list, shape_list):
                    obstacle = obs_class(id=id, state=state, vertex=vertex, **kwargs) 
                    self.obs_poly_list.append(obstacle)
                    self.obs_list.append(obstacle)

            elif obs_class.obstacle_type == 'obstacle_block':
                self.obs_block_list = []

                for id, center, shape in zip(range(number), state_list, shape_list):
                    obstacle = obs_class(id=id, center=center, length=shape[0], width=shape[1], step_time=self.step_time, **kwargs)
                    self.obs_block_list.append(obstacle)
                    self.obs_list.append(obstacle)        

            elif obs_class.obstacle_type == 'obstacle_map':
                pass
    
    def init_distribute(self, number, mode='manual', states=[[0, 0, 0]], rlow=[0, 0, 0], rhigh=[10, 10, 3.14], distance=1, random_bear=False, random_shape=False, shapes_low=0.1, shapes_high=1, **kwargs):

        if self.obs_class.obstacle_type == 'obstacle_circle':
            shapes = kwargs.get('shapes', [0.2])  # radius
            shapes_low = kwargs.get('shapes_low', 0.1)  # radius
            shapes_high = kwargs.get('shapes_high', 1)  # radius

        elif self.obs_class.obstacle_type == 'obstacle_polygon':
            shapes = kwargs.get('shapes', [[1, 2], [3, 1.5], [4, 3], [2, 4]] ) # vertex points
            
        elif self.obs_class.obstacle_type == 'obstacle_block':
            shapes = kwargs.get('shapes', [[0.5, 0.4]]) # length width
            shapes_low = kwargs.get('shapes_low', [0.1, 0.1])  # radius
            shapes_high = kwargs.get('shapes_high', [1, 1])  # radius

        shape_list = self.extend_list(shapes, number) 

        if mode == 'manual':
            state_list = self.extend_list(states, number)
        
        elif mode == 'random':
            state_list = random_points(number, np.c_[rlow], np.c_[rhigh], distance)
        
        if random_shape:
            shape_list = random_value(number, shapes_low, shapes_high)

        if random_bear:
            for state in state_list:
                state[2, 0] = np.random.uniform(low = -pi, high = pi)

        return state_list, shape_list

    def move(self):
        if self.dynamic and self.number > 0: 
            [obs.move_dynamic(**self.dynamic) for obs in self.obs_cir_list]
           
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

    @staticmethod
    def extend_list(input_list, number):

        if len(input_list) < number: 
            input_list.extend([input_list[-1]]* (number - len(input_list)) )

        return input_list
