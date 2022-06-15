from ir_sim2.world import RobotDiff
from ir_sim2.world import RobotAcker
import numpy as np

class EnvRobot:
    # a group of robots
    def __init__(self, robot_class, number=0, distribute='manual', step_time=0.1, **kwargs):

        self.number = number
        self.robot_class = robot_class
        self.type = robot_class.robot_type
        self.robot_list = []
        self.step_time = step_time

        self.distribute = distribute # 'manual', 'circular', 'random', 'opposite'
        
        if distribute == 'manual':
            # line
            state_list = kwargs.get('state_list', np.arange(number))
            goal_list = kwargs.get('goal_list', np.arange(number)[::-1])
            radius_list = kwargs.get('radius_list', [0.2] * number)
            radius_exp = kwargs.get('radius_list', [0.1] * number)

            if isinstance(radius_list, float): radius_list = [radius_list] * number
            if isinstance(radius_exp, float): radius_exp = [radius_exp] * number
        else:
            pass
        
        if number > 0:
            if robot_class.robot_shape == 'circle':
                for id, radius, state, goal in zip(range(number), radius_list, state_list, goal_list):
                    robot = robot_class(id=id, state=state, goal=goal, radius=radius, **kwargs)
                    self.robot_list.append(robot) 
            elif robot_class.robot_shape == 'rectangle':
                pass
            #     robot = robot_class(id=id, step_time=step_time, **kwargs)  

    def init_distribute(self, number, distribute='line'):
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
    
    def plot(self, ax, **kwargs):
        for robot in self.robot_list:
            robot.plot(ax, **kwargs)
    
    def plot_clear(self, ax):
        for robot in self.robot_list:
            robot.plot_clear(ax)
