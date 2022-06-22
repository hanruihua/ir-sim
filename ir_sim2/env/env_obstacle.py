import numpy as np

class EnvObstacle:

    def __init__(self, obs_class, number=1, distribute='manual', dynamic=False, step_time=0.1, sport='default', **kwargs):

        self.obs_class = obs_class
        self.number = number
        self.distribute = distribute
        self.dynamic = dynamic
        self.step_time = step_time
        self.sport = sport # default, wander, patrol

        if obs_class.obstacle_type == 'obstacle_circle':
            
            self.obs_cir_list = []

            if distribute == 'manual':
                point_list = kwargs.get('point_list', None)
                goal_list = kwargs.get('goal_list', None)
                radius_list = kwargs.get('radius_list', [0.2] * number)

                if isinstance(radius_list, float): radius_list = [radius_list] * number

            if number > 0:
                for id, radius, point, goal in zip(range(number), radius_list, point_list, goal_list):
                    obstacle = obs_class(id=id, point=point, goal=goal, radius=radius, step_time=self.step_time, **kwargs)
                    self.obs_cir_list.append(obstacle)
                    
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

        collision = False

        if self.obs_class.obstacle_type == 'obstacle_circle':
            for obs in self.obs_cir_list:
                if obs.collision_check_point(point):
                    collision = True
        
        return collision

    def plot(self, ax, **kwargs):
        if self.obs_class.obstacle_type == 'obstacle_circle':
            for obs in self.obs_cir_list:
                obs.plot(ax, **kwargs)
        
    def plot_clear(self):
        if self.obs_class.obstacle_type == 'obstacle_circle':
            for obs in self.obs_cir_list:
                obs.plot_clear()


