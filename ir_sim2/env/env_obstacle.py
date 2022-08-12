from operator import length_hint
import numpy as np

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
                for id, state, vertex in zip(range(number), state_list, vertex_list):
                    obstacle = obs_class(id=id, state=state, vertex=vertex, **kwargs) 
                    self.obs_poly_list.append(obstacle)
                    self.obs_list.append(obstacle)

        elif obs_class.obstacle_type == 'obstacle_block':
            self.obs_block_list = []

            if distribute == 'manual':
                center_list = kwargs.get('center_list', None)
                length_list = kwargs.get('length_list', None)
                width_list = kwargs.get('width_list', None)

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



