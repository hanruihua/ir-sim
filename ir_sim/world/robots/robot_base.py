import logging
import numpy as np
from math import inf, pi, atan2, sin, cos, sqrt
from collections import namedtuple
from ir_sim.util.util import get_transform
from ir_sim.util import collision_dectection_geo as cdg 
from ir_sim.world.sensors.lidar import lidar2d
from ir_sim.world.sensors.GPS import GPS
import matplotlib as mpl
from ir_sim.util.util import WrapToRegion
from ir_sim.global_param import world_param, env_param


# define geometry point and segment for collision detection.
# point [x, y]
# segment [point1, point2]
# polygon [point1, point2, point3...]
# circle [x, y, r]
point_geometry = namedtuple('point', ['x', 'y'])
circle_geometry = namedtuple('circle', ['x', 'y', 'r'])

robot_info = namedtuple('robot', 'state, velocity shape G h cone_type vel_min vel_max acce') # robot information

class RobotBase:

    robot_type = 'diff'  # omni, acker
    appearance = 'circle'  # shape list: ['circle', 'rectangle', 'polygon']
    state_dim = (3, 1) # the state dimension 
    vel_dim = (2, 1)  # the velocity dimension 
    goal_dim = (3, 1) # the goal dimension 
    position_dim=(2,1) # the position dimension 
    
    def __init__(self, id, state, vel, goal=np.zeros(goal_dim), step_time=0.1, vel_min=[-inf, -inf], vel_max=[inf, inf], acce=[inf, inf], angle_range=[-pi, pi], **kwargs):

        """
        type = 'diff', 'omni', 'ackermann' 
        """
        self.id = int(id)
        self.step_time = step_time
        self.name = 'Robot' + str(self.id)

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
        self.center = self.init_state[0:2]

        assert self.state.shape == self.state_dim and self.vel.shape == self.vel_dim and self.goal.shape == self.goal_dim

        self.angle_range = angle_range
        self.vel_min = np.around(np.c_[vel_min], 2)
        self.vel_max = np.around(np.c_[vel_max], 2)
        self.acce = np.c_[acce]

        self.vel_acce_min = np.maximum(self.vel_min, self.vel - self.acce * self.step_time) 
        self.vel_acce_max = np.minimum(self.vel_max, self.vel + self.acce * self.step_time)

        self.arrive_mode = kwargs.get('arrive_mode', 'position') # 'state', 'position'
        self.goal_threshold = kwargs.get('goal_threshold', 0.1)

        # self.collision_threshold = kwargs.get('collision_threshold', 0.001)

        # flag
        self.arrive_flag = False
        self.collision_flag = False
        self.stop_flag = False

        # collision
        self.collision_position = None

        # noise
        self.noise = kwargs.get('noise', False)
        # self.mean =  

        # Generalized inequalities for init position 
        if self.appearance == 'circle':
            self.cone_type = 'norm2'
        else:
            self.cone_type = 'Rpositive'
        
        self.G, self.h = self.gen_inequal()

        # sensor
        sensor_kwargs = kwargs.get('sensor', [])
        self.sensors = []

        for kwargs in sensor_kwargs:
            sensor_id = kwargs.get('id', None)
            if sensor_id is None or sensor_id == self.id:

                if kwargs['type'] == 'lidar':
                    self.lidar = lidar2d(robot_state=self.state, **kwargs)
                    self.sensors.append(self.lidar)
                elif kwargs['type'] == 'gps':
                    self.gps = GPS(robot_state=self.state, **kwargs)
                    self.sensors.append(self.gps)
        # plot
        self.plot_patch_list = []
        self.plot_line_list = []
        self.plot_text_list = []

        # self.alpha = kwargs.get('alpha', [0.03, 0, 0, 0.03, 0, 0])
        # self.control_std = kwargs.get('control_std', [0.01, 0.01])

    def move(self, vel, stop=True, **kwargs):
        """ vel: velocity to control the robot
            stop: the robot will stop when arriving at the goal

            return: 
        """

        if isinstance(vel, list): vel = np.c_[vel]
            
        assert vel.shape == self.vel_dim and self.state.shape == self.state_dim

        vel = np.around(vel.astype(float), 2)  # make sure the vel is float
        vel = self.range_check(vel, self.vel_acce_max, self.vel_acce_min)

        if self.stop_flag:
            vel = np.zeros(self.vel_dim)

        self.vel = vel
        self.trajectory.append(self.state)

        new_state = self.dynamics(self.state, vel, **kwargs)

        collision_flag, self.collision_position = self.collision_check(new_state, self.state)

        if collision_flag:
            
            if world_param.collision_mode == 'stop':
                self.stop_flag = True
                if self.collision_position is not None:
                    new_state[0:2] = self.collision_position[0:2]

                if not self.collision_flag: print('Collision with obstacles, robot stop move !')

            elif world_param.collision_mode == 'react':
                if self.collision_position is None:
                    self.stop_flag = True
                    print('No react mode for rectangle robot')
                else:
                    new_state[0:2] = self.collision_position[0:2]
            else:
                self.stop_flag = True
                print('wrong collision mode')
            
            self.collision_flag = collision_flag
            
        if self.arrive_flag and stop:
            self.stop_flag = True
            
        self.state = new_state
        self.center = self.state[0:2]

        self.vel_acce_min = np.around(np.maximum(self.vel_min, self.vel - self.acce * self.step_time), 2)
        self.vel_acce_max = np.around(np.minimum(self.vel_max, self.vel + self.acce * self.step_time), 2)

        self.mid_process()

        self.sensor_step()
        self.post_process()
        self.arrive()
        
    def update_info(self, state, vel):
        # update the information of the robot manually
        self.state = state
        self.vel = vel
    
    def sensor_step(self):
        # for sensor
        for sensor in self.sensors:
            sensor.step(self.state[0:3], )

    def mid_process(self):
        if self.angle_range is not None and self.state_dim[0] > 2:
            self.state[2, 0] = WrapToRegion(self.state[2, 0], self.angle_range)

    def post_process(self):
        pass

    def arrive(self):
        if self.arrive_mode == 'position':
            if np.linalg.norm(self.state[0:self.position_dim[0]] - self.goal[0:self.position_dim[0]]) <= self.goal_threshold:
                if not self.arrive_flag: logging.info('robot %d arrive at the goal', self.id)
                self.arrive_flag = True

        elif self.arrive_mode == 'state':
            if np.linalg.norm(self.state[0:self.goal_dim[0]] - self.goal) <= self.goal_threshold:
                
                if not self.arrive_flag: logging.info('robot %d arrive at the goal', self.id)
                self.arrive_flag = True

    def range_check(self, value, max, min, name='velocity'):
        
        if (value < min).any():
            print('warning: ', name + ' ' + str(value) + ' is clipped by the minimum value', np.squeeze(min).tolist())

        if (value > max).any():
            print('warning: ', name + ' ' + str(value) + ' is clipped by the maximum value', np.squeeze(max).tolist())
        
        return np.clip(value, min, max)

    # collision_check

    # def collision_check(self, state):

    #     collision_flag = False
    #     collision_position = None

    #     obs_list = env_param.obstacle_list.copy()
    #     robot_list = env_param.robot_list.copy()
    #     other_robot_list = [robot for robot in robot_list if robot.id != self.id]

    #     obj_list = other_robot_list + obs_list

    #     for obj in obj_list:
    #         collision_flag1, collision_position1 = self.collision_check_state(state, obj)

    #         if collision_flag1:
                
    #             collision_flag = True

    #             state[0:2] = collision_position1

    #             for obj in obj_list:
    #                 collision_flag2, collision_position2 = self.collision_check_state(state, obj)

    #                 if collision_flag2:
    #                     collision_position = collision_position2

    #     return collision_flag, collision_position
    
    def collision_check_single(self, state, pre_state=None):

        collision_flag = False
        collision_position = None

        # collision check grid map
        if env_param.grid_map is not None:
            collision_flag, collision_position = self.collision_check_map(state, pre_state)
            if collision_flag: return collision_flag, collision_position

        obs_list = env_param.obstacle_list.copy()
        robot_list = env_param.robot_list.copy()
        other_robot_list = [robot for robot in robot_list if robot.id != self.id]

        obj_list = other_robot_list + obs_list

        for obj in obj_list:
            collision_flag, collision_position = self.collision_check_state(state, obj)

            if collision_flag:
                return collision_flag, collision_position
            
        return collision_flag, collision_position
    
    def collision_check(self, state, pre_state=None):

        collision_flag, collision_position = self.collision_check_single(state, pre_state)

        if collision_flag:
            
            if collision_position is None:
                return True, None

            new_collision_flag = True

            cur_state = state

            loop = 0
            while new_collision_flag and loop < 10:

                cur_state[0:2] = collision_position
                flag, position = self.collision_check_single(cur_state, pre_state)

                collision_position = position
                new_collision_flag = flag
                loop += 1
                
            return collision_flag, cur_state
        
        return collision_flag, collision_position

    

    # def collision_check_object(self, obj):

    #     if self.appearance == 'circle':
    #         robot_circle = circle_geometry(self.state[0, 0], self.state[1, 0], self.radius)  
    #         if obj.appearance == 'circle':
    #             obj_circle = circle_geometry(obj.center[0, 0], obj.center[1, 0], obj.radius)  
                
    #             collision_flag, collision_position =  cdg.collision_cir_cir(robot_circle, obj_circle)

    #             if collision_flag:
    #                 if not self.collision_flag: 
    #                     logging.info('robot id %d collision', self.id) 
    #                     self.collision_position = collision_position

    #                 self.collision_flag = True
                    
    #                 return True
                    
    #         if obj.appearance == 'polygon' or obj.appearance == 'rectangle':
    #             obj_poly = [ point_geometry(v[0], v[1]) for v in obj.vertex.T]
    #             collision_flag, collision_position = cdg.collision_cir_poly(robot_circle, obj_poly)

    #             if collision_flag:
    #                 if not self.collision_flag: 
    #                     logging.info('robot id %d collision', self.id)
    #                     self.collision_position = collision_position

    #                 self.collision_flag = True
                    
    #                 return True
            
    #         if obj.appearance == 'segment':
    #             obj_seg = [point_geometry(obj.points[0][0, 0], obj.points[0][1, 0]), point_geometry(obj.points[1][0, 0], obj.points[1][1, 0])]
    #             collision_flag, collision_position = cdg.collision_cir_seg(robot_circle, obj_seg)

    #             if collision_flag:
    #                 if not self.collision_flag: 
    #                     logging.info('robot id %d collision', self.id)
    #                     self.collision_position = collision_position

    #                 self.collision_flag = True
                    
    #                 return True
        
    #     # ackermann robot
    #     if self.appearance == 'polygon' or self.appearance == 'rectangle':
    #         robot_poly = [ point_geometry(v[0], v[1]) for v in self.vertex.T]

    #         if obj.appearance == 'circle':
    #             obj_circle = circle_geometry(obj.center[0, 0], obj.center[1, 0], obj.radius)

    #             collision_flag, collision_position = cdg.collision_cir_poly(obj_circle, robot_poly)

    #             if collision_flag:
    #                 if not self.collision_flag: 
    #                     logging.info('robot id %d collision', self.id)
    #                     self.collision_position = collision_position

    #                 self.collision_flag = True
    #                 return True
            
    #         if obj.appearance == 'polygon' or obj.appearance == 'rectangle':
    #             obj_poly = [ point_geometry(v[0], v[1]) for v in obj.vertex.T]
    #             collision_flag, collision_position = cdg.collision_poly_poly(robot_poly, obj_poly)
    #             if collision_flag:
    #                 if not self.collision_flag: 
    #                     logging.info('robot id %d collision', self.id)
    #                     self.collision_position = collision_position

    #                 self.collision_flag = True
                    
    #                 return True

    #     self.collision_flag = False

    #     return False

    def collision_check_state(self, state, obj):

        collision_flag = False
        collision_position = None

        if self.appearance == 'circle':
            robot_circle = circle_geometry(state[0, 0], state[1, 0], self.radius)  
            if obj.appearance == 'circle':
                obj_circle = circle_geometry(obj.center[0, 0], obj.center[1, 0], obj.radius)  
                
                collision_flag, collision_position =  cdg.collision_cir_cir(robot_circle, obj_circle)

                if collision_flag:
                    if not self.collision_flag: logging.info('robot id %d collision', self.id) 
                        
                    return collision_flag, collision_position
                    
            if obj.appearance == 'polygon' or obj.appearance == 'rectangle':
                obj_poly = [ point_geometry(v[0], v[1]) for v in obj.vertex.T]
                collision_flag, collision_position = cdg.collision_cir_poly(robot_circle, obj_poly)

                if collision_flag:
                    if not self.collision_flag: logging.info('robot id %d collision', self.id)

                    return collision_flag, collision_position

            if obj.appearance == 'segment':
                obj_poly = [ point_geometry(v[0], v[1]) for v in obj.vertex.T]
                collision_flag, collision_position = cdg.collision_cir_poly(robot_circle, obj_poly)
                if collision_flag:
                    if not self.collision_flag: 
                        logging.info('robot id %d collision', self.id)
                        self.collision_position = collision_position

                    self.collision_flag = True
                    
                    return collision_flag, collision_position

        # ackermann robot
        if self.appearance == 'polygon' or self.appearance == 'rectangle':
            vertex = self.calcuate_vertex(state)
            robot_poly = [ point_geometry(v[0], v[1]) for v in vertex.T]

            if obj.appearance == 'circle':
                obj_circle = circle_geometry(obj.center[0, 0], obj.center[1, 0], obj.radius)

                collision_flag, collision_position = cdg.collision_poly_cir(robot_poly, obj_circle)

                if collision_flag:
                    if not self.collision_flag: logging.info('robot id %d collision', self.id)
                        
                    return collision_flag, collision_position
            
            if obj.appearance == 'polygon' or obj.appearance == 'rectangle':
                obj_poly = [ point_geometry(v[0], v[1]) for v in obj.vertex.T]
                collision_flag, collision_position = cdg.collision_poly_poly(robot_poly, obj_poly)
                if collision_flag:
                    if not self.collision_flag: logging.info('robot id %d collision', self.id)
                        
                    return collision_flag, collision_position

            if obj.appearance == 'segment':
                obj_poly = [ point_geometry(v[0], v[1]) for v in obj.vertex.T]
                collision_flag, collision_position = cdg.collision_poly_poly(robot_poly, obj_poly)
                if collision_flag:
                    if not self.collision_flag: 
                        logging.info('robot id %d collision', self.id)
                        self.collision_position = collision_position

                    self.collision_flag = True
                    
                    return collision_flag, collision_position

        return collision_flag, collision_position


    def collision_check_map(self, state, pre_state):
        map_obstacle_positions = env_param.map_obstacle_positions
        # obstacle_position_array = 
        collision_flag_array = self.collision_check_array_state(map_obstacle_positions, state)
        collision_flag = np.any(collision_flag_array==True)

        if collision_flag:
            collision_index = np.where(collision_flag_array==True)
            distance_list = [RobotBase.distance(state[0:2], map_obstacle_positions[0:2, p:p+1] ) for p in collision_index[0]]
            
            min_dis = min(distance_list)
            min_ind = distance_list.index(min_dis)
            obs_ind = collision_index[0][min_ind]
            min_obs_position = map_obstacle_positions[0:2, obs_ind:obs_ind+1]

            if self.appearance == 'circle':
                unit_diff = (state[0:2] - min_obs_position) / min_dis
                colision_position = min_obs_position + (self.radius + 0.0001) * unit_diff
            elif self.appearance == 'rectangle':
                colision_position = pre_state[0:2]

        else:
            colision_position = None
        # if collision_flag:
        #     collision_index = np.where(collision_flag_array==True)
        #     # collision_positions = map_obstacle_positions[0:2, collision_index[0]]
        #     # collision_position = np.expand_dims(collision_position, axis=1)
        #     # temp = collision_position
        #     # a = 1
        # else:
        #     collision_position = None
        
        return collision_flag, colision_position


    def collision_check_point(self, point):
        # utilize the generalized inequality to judge the collision with a point
        assert point.shape == self.position_dim

        trans, rot = get_transform(self.state)
        trans_point = np.linalg.inv(rot) @ ( point - trans)

        return RobotBase.InCone(self.G @ trans_point - self.h, self.cone_type)

    def collision_check_array(self, point_array):
        # point_array: (2* number)
        trans, rot = get_transform(self.state)
        trans_array = np.linalg.inv(rot) @ ( point_array - trans)

        temp = self.G @ trans_array - self.h

        if self.cone_type == 'Rpositive':
            collision_matirx = np.all(temp <= 0, axis=0)
        elif self.cone_type == 'norm2':
            collision_matirx = np.squeeze(np.linalg.norm(temp[0:-1], axis=0) - temp[-1]) <= 0

        return collision_matirx
    
    def collision_check_array_state(self, point_array, state):
        # point_array: (2* number)
        trans, rot = get_transform(state)
        trans_array = np.linalg.inv(rot) @ ( point_array - trans)

        temp = self.G @ trans_array - self.h

        if self.cone_type == 'Rpositive':
            collision_matirx = np.all(temp <= 0, axis=0)
        elif self.cone_type == 'norm2':
            collision_matirx = np.squeeze(np.linalg.norm(temp[0:-1], axis=0) - temp[-1]) <= 0

        return collision_matirx


    def reset(self):
        self.state = self.init_state
        self.center = self.init_state[0:2]
        self.goal = self.init_goal_state
        self.vel = self.init_vel

        self.collision_flag = False
        self.arrive_flag = False
        self.stop_flag = False
        self.trajectory = []

    def dynamics(self, vel):

        """ vel: the input velocity
            return: the next state
        """
        raise NotImplementedError

    def gen_inequal(self):
        # Calculate the matrix G and g for the Generalized inequality: G @ point <_k h,  at the init position
        # self.G, self.h = self.gen_inequal()

        if self.appearance == 'circle':  
            G = np.array([ [1, 0], [0, 1], [0, 0] ])
            h = np.array( [ [0], [0], [-self.radius] ] ) 
        else:
            num = self.init_vertex.shape[1]

            G = np.zeros((num, 2)) 
            h = np.zeros((num, 1)) 
        
            for i in range(num):
                if i + 1 < num:
                    pre_point = self.init_vertex[:, i]
                    next_point = self.init_vertex[:, i+1]
                else:
                    pre_point = self.init_vertex[:, i]
                    next_point = self.init_vertex[:, 0]
                
                diff = next_point - pre_point
                
                a = diff[1]
                b = -diff[0]
                c = a * pre_point[0] + b * pre_point[1]

                G[i, 0] = a
                G[i, 1] = b
                h[i, 0] = c 
        
        return G, h
    
    def gen_inequal_global(self):
        # Calculate the matrix G and g for the Generalized inequality: G @ point <_k g,  at the current position
        # G, h = self.gen_inequal()
        # 
        raise NotImplementedError
    
    def cal_des_vel(self):
        # calculate the desired velocity
        raise NotImplementedError

    def get_info(self):
        # return the information of the robot: 'state, velocity shape G g cone_type vel_min vel_max acce'
        return robot_info(self.state, self.vel, self.shape, self.G, self.h, self.cone_type, self.vel_min, self.vel_max, self.acce)
        
    def get_edges(self):

        edge_list = []
        ver_num = self.vertex.shape[1]

        for i in range(ver_num):
            if i < ver_num - 1:
                edge = [ self.vertex[:, i], self.vertex[:, i+1] ]
            else:
                edge = [ self.vertex[:, i], self.vertex[:, 0] ]

            edge_list.append(edge)

        return edge_list

    def get_lidar_scan(self):
        return self.lidar.get_LaserScan()

    def get_landmarks(self):
        return self.lidar.get_landmarks()
    
    def get_obstacles(self):
        return self.lidar.get_obstacles()

    def plot(self, ax, show_sensor=True, **kwargs):
        # plot the robot in the map
        self.plot_robot(ax, **kwargs)
        if show_sensor: self.plot_sensors(ax, **kwargs)
            
    def plot_robot(self, ax, robot_color = 'g', goal_color='r', show_goal=True, show_text=False, show_traj=False, traj_type='-g', fontsize=10, **kwargs):

        # default: plot circle

        x = self.state[0, 0]
        y = self.state[1, 0]
        
        goal_x = self.goal[0, 0]
        goal_y = self.goal[1, 0]

        robot_circle = mpl.patches.Circle(xy=(x, y), radius = self.radius, color = robot_color)
        robot_circle.set_zorder(3)

        ax.add_patch(robot_circle)
        if show_text: 
            r_text = ax.text(x - 0.5, y, 'r'+ str(self.id), fontsize = fontsize, color = 'r')
            self.plot_text_list.append(r_text)
        self.plot_patch_list.append(robot_circle)

        if show_goal:
            goal_circle = mpl.patches.Circle(xy=(goal_x, goal_y), radius = self.radius, color=goal_color, alpha=0.5)
            goal_circle.set_zorder(1)
        
            ax.add_patch(goal_circle)
            if show_text: 
                g_text = ax.text(goal_x + 0.3, goal_y, 'g'+ str(self.id), fontsize = fontsize, color = 'k')
                self.plot_text_list.append(g_text)
            self.plot_patch_list.append(goal_circle)

        if show_traj:
            x_list = [t[0, 0] for t in self.trajectory]
            y_list = [t[1, 0] for t in self.trajectory]
            self.plot_line_list.append(ax.plot(x_list, y_list, traj_type))


    def plot_sensors(self, ax, **kwargs):
        for sensor in self.sensors:
            plot_line_list, plot_patch_list = sensor.plot(ax)

            self.plot_patch_list += plot_patch_list
            self.plot_line_list += plot_line_list

    def plot_clear(self, ax):
        [patch.remove() for patch in self.plot_patch_list]
        [line.pop(0).remove() for line in self.plot_line_list]
        [text.remove() for text in self.plot_text_list]

        self.plot_patch_list = []
        self.plot_line_list = []
        self.plot_text_list = []

    @staticmethod
    def InCone(point, cone_type='Rpositive'):
        if cone_type == 'Rpositive':
            return (point<=0).all()
        elif cone_type == 'norm2':
            return np.squeeze(np.linalg.norm(point[0:-1]) - point[-1]) <= 0
    
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
    
    @staticmethod
    def distance(position1, position2):
        return sqrt( (position2[0, 0] - position1[0, 0]) **2 + (position2[1, 0] - position1[1, 0]) **2 )
    
    







    
