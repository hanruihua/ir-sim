import numpy as np
from math import inf
import logging

class robot_base:
    def __init__(self, id, shape='circle', model_type='diff', state_dim=(3,1), vel_dim=(2, 1), goal_dim=(3, 1), position_dim=(2,1), step_time=0.1, **kwargs):

        """
            type = 'diff', 'omni', 'ackermann' 
        """

        self.id = int(id)
        self.type = model_type
        self.step_time = step_time

        self.state_dim = state_dim
        self.vel_dim = vel_dim
        self.position_dim = position_dim

        self.init_state = kwargs.get('state', np.zeros(state_dim))
        self.init_vel = kwargs.get('vel', np.zeros(vel_dim))
        self.init_goal_state = kwargs.get('goal', np.ones(state_dim))

        if isinstance(self.init_state, list): 
            self.init_state = np.array(init_state, ndmin=2).T

        if isinstance(self.init_vel, list): 
            self.init_vel = np.array(self.init_vel, ndmin=2).T

        if isinstance(self.init_goal_state, list): 
            self.init_goal_state = np.array(self.init_goal_state, ndmin=2).T

        self.state = self.init_state
        self.goal = self.init_goal_state
        self.vel = self.init_vel

        self.shape = shape   # shape list: ['circle', 'rectangle', 'polygon']
        self.arrive_mode = kwargs.get('arrive_mode', 'position') # 'state', 'position'
        
        self.vel_limit = kwargs.get('vel_limit', [-inf, inf])
        self.goal_threshold = kwargs.get('goal_threshold', 0.1)
        
        # flag
        self.arrive_flag = False
        self.collision_flag = False
        self.cone = None

        # noise
        self.noise = kwargs.get('noise', False)

        # # sensor
        # lidar_args = kwargs.get('lidar2d', None)

        # if lidar_args is not None:
        #     id_list = lidar_args['id_list']

        # if lidar_args is not None and self.id in id_list:
        #     self.lidar = lidar2d(**lidar_args)
        # else:
        #     self.lidar = None
        

        # self.alpha = kwargs.get('alpha', [0.03, 0, 0, 0.03, 0, 0])
        # self.control_std = kwargs.get('control_std', [0.01, 0.01])

    def move(self, vel, stop=True, **kwargs):
        """ vel: velocity to control the robot
            stop: the robot will stop when arriving at the goal

            return: 
        """
        if isinstance(vel, list): 
            vel = np.array(vel, ndmin=2).T
        
        assert vel.shape == self.vel_dim

        if vel[0, 0] < self.vel_limit[0] or vel[1, 0] > self.vel_limit[1]:
            vel = np.clip(vel, self.vel_limit[0], self.vel_limit[1])
            logging.warning("The velocity is clipped within the limit")
            
        if stop:
            if self.arrive_flag or self.collision_flag:
                vel = np.zeros(self.vel_dim)

        self.dynamics(vel, **kwargs)
        self.arrive_flag = self.arrive()
    
    def update_info(self, state, vel):
        # update the information of the robot manually
        self.state = state
        self.vel = vel
    
    def dynamics(self, vel):

        """ vel: the input velocity
            return: the next state
        """
        raise NotImplementedError

    def arrive(self):
        if self.arrive_mode == 'position':
            return np.linalg.norm(self.state[0:self.position_dim[0]] - self.goal[0:self.position_dim[0]]) <= self.goal_threshold
        elif self.arrive_mode == 'state':
            return np.linalg.norm(self.state - self.goal) <= self.goal_threshold

        
    def collision_check_point():
        pass

    def collision_check_opt():
        pass




    
