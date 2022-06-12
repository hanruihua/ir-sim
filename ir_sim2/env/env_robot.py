from ir_sim2.world import RobotDiff
from ir_sim2.world import RobotAcker
import numpy as np

class EnvRobot:
    # a group of robots
    def __init__(self, robot_class, robot_number=1, distribute_mode='manual', step_time=0.1, **kwargs):

        self.robot_number = robot_number
        self.robot_class = robot_class
        self.robot_type = robot_class.robot_type
        self.robot_list = []
        self.step_time = step_time

        self.dis_mode = distribute_mode # 'manual', 'circular', 'random', 'opposite'
        
        if robot_number > 0:
            if distribute_mode == 'manual':
                # init_state_lis
                # if 'radius_list' and 'init_state_list' and 'goal_list' in kwargs.keys():
                init_state_list = kwargs.get('init_state_list', np.arange(robot_number))
            else:
                pass
            
        for id in range(robot_number):
            # id, shape='circle', step_time=0.1, radius=0.2, radius_exp=0.1, vel_min=[-2, -2], vel_max=[2, 2], 
            if robot_class.robot_shape == 'circle':
                robot = robot_class(id=id, step_time=step_time, radius=radius_list[i], **kwargs)   
            elif robot_class.robot_shape == 'rectangle':
                robot = robot_class(id=id, step_time=step_time, **kwargs)  

            self.robot_list.append(robot)

    def init_distribute(self, number, distribute_mode='opposite'):
        pass


    def collision_check(self):
        pass

    def move(self, vel_list=[], **vel_kwargs):

        # vel_kwargs: 
        #   diff:
        #       vel_type = 'diff', 'omni'
        #       stop=True, whether stop when arrive at the goal
        #       noise=False, 
        #       alpha = [0.01, 0, 0, 0.01, 0, 0], noise for diff
        #   omni:
        #       control_std = [0.01, 0.01], noise for omni

        for robot, vel in zip(self.robot_list, vel_list):
            robot.move(vel, **vel_kwargs)

        
