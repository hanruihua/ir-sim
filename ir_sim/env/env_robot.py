import numpy as np
from math import pi, sin, cos
from ir_sim.util.util import WrapToPi, random_points, random_value

class EnvRobot:
    # a group of robots
    def __init__(self, robot_class, number=0, distribute=dict(), step_time=0.01, **kwargs):

        self.number = number
        self.robot_class = robot_class
        self.type = robot_class.robot_type
        self.appearance = robot_class.appearance
        self.robot_list = []
        self.step_time = step_time
        self.state_dim = robot_class.state_dim
        
        if number > 0:
            if number == 1:
                robot = robot_class(id=0, step_time=self.step_time, **kwargs)
                self.robot_list.append(robot)
            else:
                state_list, goal_list, shape_list = self.init_distribute(number, **distribute)

                if robot_class.appearance == 'circle':
                    for id, radius, state, goal in zip(range(number), shape_list, state_list, goal_list):
                        
                        kwargs['state'], kwargs['goal'], kwargs['radius'] = state, goal, radius
                        robot = robot_class(id=id, step_time=self.step_time, **kwargs)
                        self.robot_list.append(robot)

                elif robot_class.appearance == 'rectangle':
                    for id, shape, state, goal in zip(range(number), shape_list, state_list, goal_list): 
                        kwargs['state'], kwargs['goal'], kwargs['shape'] = state, goal, shape
                        robot = robot_class(id=id, step_time=self.step_time, **kwargs)
                        self.robot_list.append(robot)
   
    def init_distribute(self, number, mode='manual', states=[[0, 0, 0]], goals=[[1, 1, 0]], circle=[5, 5, 3], rlow=[0, 0, 0], rhigh=[10, 10, 3.14], distance=1, random_bear=False, random_shape=False, radius_low=0.1, radius_high=1, **kwargs):

        # multiple robots distribution

        # default shapes
        if self.appearance == 'circle':
            shapes = kwargs.get('shapes', [0.2])
        elif self.appearance == 'rectangle':
            shapes = kwargs.get('shapes', [[4.6, 1.6, 3, 1.6]])

        shape_list = self.extend_list(shapes, number) 
        
        if mode == 'manual':
            state_list = self.extend_list(states, number)
            goal_list = self.extend_list(goals, number) 

        elif mode == 'circular':
            cx, cy, cr = circle[0:3]  # x, y, radius
            theta_space = np.linspace(0, 2*pi, number, endpoint=False)
            
            if self.state_dim == (3, 1):
                state_list = [ np.array([ [cx + cos(theta) * cr], [cy + sin(theta) * cr], [WrapToPi(theta + pi)] ]) for theta in theta_space]

                goal_list = [ np.array([ [cx + cos(theta + pi) * cr], [cy + sin(theta + pi) * cr], [WrapToPi(theta + pi)]]) for theta in theta_space]
    
            elif self.state_dim == (4, 1):
                state_list = [ np.array([ [cx + cos(theta) * cr], [cy + sin(theta) * cr], [WrapToPi(theta + pi)], [0]]) for theta in theta_space]

                goal_list = [ np.array([ [cx + cos(theta + pi) * cr], [cy + sin(theta + pi) * cr], [WrapToPi(theta + pi)]]) for theta in theta_space]

            elif self.state_dim == (2, 1):
                state_list = [ np.array([ [cx + cos(theta) * cr], [cy + sin(theta) * cr] ]) for theta in theta_space]
                goal_list = [ np.array([ [cx + cos(theta + pi) * cr], [cy + sin(theta + pi) * cr] ]) for theta in theta_space]
            
        elif mode == 'random':
            state_list = random_points(number, np.c_[rlow], np.c_[rhigh], distance)  # diff 3*1, acker: 4*1
            goal_list = random_points(number, np.c_[rlow[0:3]], np.c_[rhigh[0:3]], distance)  # dim 3*1

        elif mode == 'line':
            pass
        
        if random_shape and self.appearance == 'circle':
            shape_list = random_value(number, radius_low, radius_high)
        
        if random_bear:
            for state in state_list:
                state[2, 0] = np.random.uniform(low = -pi, high = pi)
        
        return state_list, goal_list, shape_list
    

    def cal_des_vel(self, **kwargs):
        return [robot.cal_des_vel(**kwargs) for robot in self.robot_list]

    # def collision_check(self, env_obstacle):

    #     for i, robot in enumerate(self.robot_list):
    #         other_robot_list = [o_robot for j, o_robot in enumerate(self.robot_list) if j != i]
    #         object_list = env_obstacle.obs_list + other_robot_list
    #         if self.collision_check_obj_list(robot, object_list):
    #             return True
            
    #     return False

    # def collision_check_list(self, env_obstacle_list):
    #     # return the list of collision flags
    #     collision_list = []
    #     obs_list = []

    #     for env_obs in env_obstacle_list:
    #         obs_list.extend(env_obs.obs_list)

    #     for i, robot in enumerate(self.robot_list):
    #         other_robot_list = [o_robot for j, o_robot in enumerate(self.robot_list) if j != i]
    #         object_list = obs_list + other_robot_list
    #         collision_list.append(self.collision_check_obj_list(robot, object_list))
        
    #     return collision_list
    
    # def collision_check_obj_list(self, robot, object_list):

    #     for obj in object_list:
    #         if robot.collision_check_object(obj):
    #             return True

    #     return False

    def arrive(self):
        return all([r.arrive_flag for r in self.robot_list])

    def arrive_list(self):
        return [r.arrive_flag for r in self.robot_list]
    
    def collision_list(self):
        return [r.collision_flag for r in self.robot_list]

    def collision_status(self):
        return any([r.collision_flag for r in self.robot_list])

    def move(self, velocity=[], vel_id=1, **vel_kwargs):
        # vel_kwargs: 
        #   diff:
        #       vel_type = 'diff', 'omni'
        #       noise=False, 
        #       alpha = [0.01, 0, 0, 0.01, 0, 0], noise for diff
        #   omni:
        #       control_std = [0.01, 0.01], noise for omni
        if not isinstance(velocity, list):
            
            if len(self.robot_list) >= 1:
                if vel_id != 0:
                    self.robot_list[vel_id-1].move(velocity, **vel_kwargs)
                else:
                    print('zero velocity id')
            # else:
            #     print('No robots')

            # sensor step
            for robot in self.robot_list:
                robot.sensor_step()
            
        else:
            for robot, vel in zip(self.robot_list, velocity):
                robot.move(vel, **vel_kwargs)
    
    
    def reset(self, id=-1):
        if id == -1:
            [robot.reset() for robot in self.robot_list]
        else:
            [robot.reset() for robot in self.robot_list if robot.id == id]

    def plot(self, ax, **kwargs):
        for robot in self.robot_list:
            robot.plot(ax, **kwargs)
    
    def plot_clear(self, ax):
        for robot in self.robot_list:
            robot.plot_clear(ax)

    @staticmethod
    def extend_list(input_list, number):

        if len(input_list) < number: 
            input_list.extend([input_list[-1]]* (number - len(input_list)) )

        return input_list