import numpy as np
from math import inf, pi, atan2
import logging

class RobotBase:

    robot_type = 'diff'  # omni, acker
    robot_shape = 'circle'  # shape list: ['circle', 'rectangle', 'polygon']
    state_dim = (3, 1) # the state dimension 
    vel_dim = (2, 1)  # the velocity dimension 
    goal_dim = (3, 1) # the goal dimension 
    position_dim=(2,1) # the position dimension 

    def __init__(self, id, state, vel, goal, step_time=0.1, **kwargs):

        """
            type = 'diff', 'omni', 'ackermann' 
        """
        self.id = int(id)
        self.step_time = step_time

        self.init_state = state
        self.init_vel = vel
        self.init_goal_state = goal 

        if isinstance(self.init_state, list): self.init_state = np.c_[self.init_state]
        if isinstance(self.init_vel, list): self.init_vel = np.c_[self.init_vel]
        if isinstance(self.init_goal_state, list): self.init_goal_state = np.c_[self.init_goal_state]

        self.state = self.init_state
        self.goal = self.init_goal_state
        self.vel = self.init_vel
        self.trajectory = []
        
        self.arrive_mode = kwargs.get('arrive_mode', 'position') # 'state', 'position'
        self.vel_min = kwargs.get('vel_min', np.c_[[-inf, -inf]])
        self.vel_max = kwargs.get('vel_max', np.c_[[inf, inf]])
        self.goal_threshold = kwargs.get('goal_threshold', 0.1)
        if isinstance(self.vel_min, list): self.vel_min = np.c_[self.vel_min]
        if isinstance(self.vel_max, list): self.vel_max = np.c_[self.vel_max]

        # flag
        self.arrive_flag = False
        self.collision_flag = False
        self.cone = None

        # noise
        self.noise = kwargs.get('noise', False)

        # Generalized inequalities
        self.G, self.g = self.gen_inequal()
        self.cone_type = 'Rpositive' # 'Rpositive'; 'norm2' 

        # sensor
        self.lidar = None
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
        if isinstance(vel, list): vel = np.c_[vel]
            
        assert vel.shape == self.vel_dim

        if (vel < self.vel_min).any() or (vel > self.vel_max).any():
            vel = np.clip(vel, self.vel_min, self.vel_max)
            logging.warning("The velocity is clipped to be %s", vel.tolist())
        if stop:
            if self.arrive_flag or self.collision_flag:
                vel = np.zeros(self.vel_dim)

        self.trajectory.append(self.state)
        self.dynamics(vel, **kwargs)
        self.arrive_flag = self.arrive()
    
    def update_info(self, state, vel):
        # update the information of the robot manually
        self.state = state
        self.vel = vel
    
    def arrive(self):
        if self.arrive_mode == 'position':
            return np.linalg.norm(self.state[0:RobotBase.position_dim[0]] - self.goal[0:RobotBase.position_dim[0]]) <= self.goal_threshold
        elif self.arrive_mode == 'state':
            return np.linalg.norm(self.state - self.goal) <= self.goal_threshold

    def collision_check_point(self, point):
        # utilize the generalized inequality to judge the collision with a point
        assert point.shape == RobotBase.position_dim
        return RobotBase.InCone(self.G @ point - self.g, self.cone_type)

        #  def inside(self, obs, point):
        #     # Ao<=b
        #     assert point.shape == (2, 1)
        #     return self.cone(obs.A @ point - obs.b, obs.cone)

        # def cone(self, point, cone='R_positive'):
        #     # cone: R_positive, norm2
        #     if cone == 'R_positive':
        #         return (point<=0).all()
        #     elif cone == 'norm2':
        #         return np.squeeze( np.linalg.norm(point[0:-1]) - point[-1] ) <= 0

        # return self.A @ point - self.b <= 

    def collision_check_opt(self):
        pass
    
    @staticmethod
    def InCone(point, cone_type='Rpositive'):
        if cone_type == 'Rpositive':
            return (point<=0).all()
        elif cone_type == 'norm2':
            return np.squeeze(np.linalg.norm(point[0:-1]) - point[-1]) <= 0

    def dynamics(self, vel):

        """ vel: the input velocity
            return: the next state
        """
        raise NotImplementedError

    def gen_inequal(self):
        # Calculate the matrix G and g for the Generalized inequality: G @ point <_k g, 
        # self.G, self.g = self.gen_inequal()
        raise NotImplementedError
    
    def cal_des_vel(self):
        # calculate the desired velocity
        raise NotImplementedError

    def robot_plot(self,):
        # plot the robot in the map
        raise NotImplementedError

    def reset(self):
        self.state = self.init_state
        self.vel = self.init_vel
        self.goal_state = self.init_goal_state

    @staticmethod
    def wraptopi(radian):

        while radian > pi:
            radian = radian - 2 * pi

        while radian < -pi:
            radian = radian + 2 * pi

        return radian

    @staticmethod
    def relative_position(position1, position2, topi=True):
        diff = position2[0:RobotBase.position_dim[0]]-position1[0:RobotBase.position_dim[0]]
        dis = np.linalg.norm(diff)
        radian = atan2(diff[1, 0], diff[0, 0])

        if topi: radian = RobotBase.wraptopi(radian)

        return dis, radian
    
    







    
