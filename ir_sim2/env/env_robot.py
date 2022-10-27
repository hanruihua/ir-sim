from ir_sim2.world import RobotDiff
from ir_sim2.world import RobotAcker
import numpy as np
from math import pi, sin, cos

class EnvRobot:
    # a group of robots
    def __init__(self, robot_class, number=0, distribute='manual', step_time=0.01, random_bear=False, **kwargs):

        self.number = number
        self.robot_class = robot_class
        self.type = robot_class.robot_type
        self.robot_list = []
        self.step_time = step_time

        self.distribute = distribute # 'manual', 'circular', 'random', 'opposite'
        self.random_bear = random_bear

        shape_list = kwargs.get('shape_list', [0.2] * number if robot_class.appearance == 'circle' else [[4.6, 1.6, 3, 1.6]]*number)
        if isinstance(shape_list, float): shape_list = [shape_list] * number
        if len(shape_list) == 1: shape_list = shape_list * number

        if number > 0:
            if distribute == 'manual':
                # if robot_class.appearance == 'circle':
                #     state_list = kwargs.get('state_list', None)
                #     goal_list = kwargs.get('goal_list', None)
                #     radius_list = kwargs.get('shape_list', [0.2] * number)
                #     radius_exp = kwargs.get('radius_exp_list', [0.1] * number)

                #     if isinstance(radius_list, float): radius_list = [radius_list] * number
                #     if isinstance(radius_exp, float): radius_exp = [radius_exp] * number

                # if robot_class.appearance == 'rectangle':
                #     state_list = kwargs.get('state_list', None)
                #     goal_list = kwargs.get('goal_list', None)
                #     shape_list = kwargs.get('shape_list', [[4.6, 1.6, 3, 1.6]]*number)
                assert 'state_list' in kwargs.keys() and 'goal_list' in kwargs.keys()
        
                state_list = kwargs['state_list']
                goal_list = kwargs['goal_list']
                
            else:
                state_list, goal_list = self.init_distribute(number, distribute, robot_class.robot_type, **kwargs)

            if robot_class.appearance == 'circle':
                for id, radius, state, goal in zip(range(number), shape_list, state_list, goal_list):
                    robot = robot_class(id=id, state=state, goal=goal, radius=radius, step_time=self.step_time, **kwargs)
                    self.robot_list.append(robot)
            elif robot_class.appearance == 'rectangle':
                for id, shape, state, goal in zip(range(number), shape_list, state_list, goal_list): 
                    robot = robot_class(id=id, state=state, goal=goal, shape=shape, step_time=self.step_time, **kwargs)
                    self.robot_list.append(robot)
            
    def init_distribute(self, number, distribute='line', robot_type='diff', **kwargs):
        
        if distribute == 'line':
            pass
        
        elif distribute == 'circular':
            cx, cy, cr = kwargs['circular']  # x, y, radius
            theta_space = np.linspace(0, 2*pi, number, endpoint=False)
            goal_list = [ np.array([ [cx + cos(theta + pi) * cr], [cy + sin(theta + pi) * cr], [theta + pi]]) for theta in theta_space]

            if robot_type == 'diff':
                state_list = [ np.array([ [cx + cos(theta) * cr], [cy + sin(theta) * cr], [theta + pi]]) for theta in theta_space]
            elif robot_type == 'acker':
                state_list = [ np.array([ [cx + cos(theta) * cr], [cy + sin(theta) * cr], [theta + pi], [0]]) for theta in theta_space]

        return state_list, goal_list
    
    def cal_des_vel(self, **kwargs):
        return [robot.cal_des_vel(**kwargs) for robot in self.robot_list]

    def collision_check(self, env_obstacle):

        for i, robot in enumerate(self.robot_list):
            other_robot_list = [o_robot for j, o_robot in enumerate(self.robot_list) if j != i]
            object_list = env_obstacle.obs_list + other_robot_list
            if self.collision_check_obj_list(robot, object_list):
                return True
            
        return False

    def collision_check_list(self, env_obstacle_list):
        # return the list of collision flags
        collision_list = []
        obs_list = []

        for env_obs in env_obstacle_list:
            obs_list.extend(env_obs.obs_list)

        for i, robot in enumerate(self.robot_list):
            other_robot_list = [o_robot for j, o_robot in enumerate(self.robot_list) if j != i]
            object_list = obs_list + other_robot_list
            collision_list.append(self.collision_check_obj_list(robot, object_list))
        
        return collision_list
    
    def collision_check_obj_list(self, robot, object_list):

        for obj in object_list:
            if robot.collision_check_object(obj):
                return True

        return False

    def arrive(self):
        return all([r.arrive_flag for r in self.robot_list])

    def arrive_list(self):
        return [r.arrive_flag for r in self.robot_list]
    
    def collision_list(self):
        return [r.collision_flag for r in self.robot_list]

    def collision(self):
        return any([r.collision_flag for r in self.robot_list])

    def move(self, velocity=[], vel_id=1, **vel_kwargs):
        # vel_kwargs: 
        #   diff:
        #       vel_type = 'diff', 'omni'
        #       stop=True, whether stop when arrive at the goal
        #       noise=False, 
        #       alpha = [0.01, 0, 0, 0.01, 0, 0], noise for diff
        #   omni:
        #       control_std = [0.01, 0.01], noise for omni
        if not isinstance(velocity, list):
            if vel_id != 0:
                self.robot_list[vel_id-1].move(velocity, **vel_kwargs)
            else:
                print('zero velocity id')

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

        #  # sensor step
        # for robot in self.robot_list:
        #     robot.sensor_step()



    def plot(self, ax, **kwargs):
        for robot in self.robot_list:
            robot.plot(ax, **kwargs)
    
    def plot_clear(self, ax):
        for robot in self.robot_list:
            robot.plot_clear(ax)
