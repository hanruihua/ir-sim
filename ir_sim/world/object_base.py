import shapely
import logging
import itertools
import numpy as np
import matplotlib as mpl

from dataclasses import dataclass
from shapely.ops import transform
from ir_sim.lib.behavior import Behavior
from math import inf, pi, atan2, cos, sin, sqrt
from ir_sim.global_param import world_param, env_param
from ir_sim.util.util import WrapToRegion, get_transform, relative_position, WrapToPi
from ir_sim.lib.generation import random_generate_polygon
from ir_sim.world.sensors.sensor_factory import SensorFactory
from shapely import Point, Polygon, LineString, minimum_bounding_radius, MultiPoint


@dataclass
class ObjectInfo:
    id: int
    shape: str
    kinematics: str
    role: str
    color: str
    static: bool
    goal: np.ndarray
    vel_min: np.ndarray
    vel_max: np.ndarray
    acce: np.ndarray
    angle_range: np.ndarray
    goal_threshold: float

    def add_property(self, key, value):
        setattr(self, key, value)


class ObjectBase:

    id_iter = itertools.count()
    vel_dim = (2, 1)

    def __init__(self, shape: str='circle', shape_tuple=None, state=[0, 0, 0], velocity=[0, 0], goal=[10, 10, 0], kinematics: str='omni', role: str='obstacle', color='k', static=False, vel_min=[-1, -1], vel_max=[1, 1], acce=[inf, inf], angle_range=[-pi, pi], behavior=None, goal_threshold=0.1, sensors=None, kinematics_dict=dict(), arrive_mode='position', description=None, group=0, reso=0.1, **kwargs) -> None:

        '''
        parameters:
        -----------
            shape: the shape of the object, a string, including: circle, polygon, linestring,

            shape_tuple: tuple to init the geometry, default is None; A sequence of (x, y) numeric coordinate pairs or triples, or an array-like with shape (N, 2)
                for circle, the list should have be: (center_x, center_y, radius)
                for polygon, the list should have the element of vertices: [vertices], number of vertices >= 3
                for lineString, composed of one or more line segments, the list should have the element of vertices: [vertices]. 

            state: the state of the object, list or numpy. default is [0, 0, 0], [x, y, theta]
            velocity: the velocity of the object, list or numpy. default is [0, 0], [vx, vy]
            kinematics: the moving kinematics of the object, including omni, diff, acker, custom; default omni, if custom, 

            static: whether static object; default False

            role: the role of the object, including: robot, obstacle, landmark, target, default is 'obstacle'

            flag: 
                stop_flag: whether the object is stopped, default False
                arrive_flag: whether the object is arrived, default False
                collision_flag: whether the object is collided, default False

            behavior: the behavior of the object, 
                dash: the object will dash to the target position, and stop when arrive the target position
                wander: the object will wander in the world (random select goal to move)
                default is dash

            arrive_mode: position or state

        '''

        self._id = next(ObjectBase.id_iter)
        self._shape = shape
        self._init_geometry = self.construct_geometry(shape, shape_tuple, reso)
        
        self._state = np.c_[state]
        self._init_state = np.c_[state]

        self._velocity = np.c_[velocity]
        self._init_velocity = np.c_[velocity]

        self._goal = np.c_[goal]
        self._init_goal = np.c_[goal]
        
        self._geometry = self.geometry_transform(self._init_geometry, self._state) 
        self.group = group

        self.kinematics = kinematics
        self.kinematics_dict = kinematics_dict
    
        # flag
        self.stop_flag = False
        self.arrive_flag = False
        self.collision_flag = False

        # information
        self.static = static
        self.vel_min = np.c_[vel_min]
        self.vel_max = np.c_[vel_max]
        self.color = color
        self.role = role
        self.info = ObjectInfo(self._id, shape, kinematics, role, color, static, np.c_[goal], np.c_[vel_min], np.c_[vel_max], np.c_[acce], np.c_[angle_range], goal_threshold)
        
        self.length = kwargs.get('length', None)
        self.width = kwargs.get('width', None)

        self.trajectory = []

        self.description = description

        # arrive judgement
        self.goal_threshold = goal_threshold
        self.arrive_mode = arrive_mode #

        # sensor
        sf = SensorFactory()
        if sensors is not None: 
            self.sensors = [sf.create_sensor(self._state[0:3], self._id, **sensor_kwargs) for sensor_kwargs in sensors] 

            self.lidar = [sensor for sensor in self.sensors if sensor.sensor_type == 'lidar'][0]
        else:
            self.sensors = []

        # behavior
        self.obj_behavior = Behavior(self.info, behavior)

        if self.obj_behavior.behavior_dict is not None and self.obj_behavior.behavior_dict['name'] == 'wander':
            range_low = np.c_[self.obj_behavior.behavior_dict['range_low']]
            range_high = np.c_[self.obj_behavior.behavior_dict['range_high']]

            self._goal = np.random.uniform(range_low, range_high)
        
        # plot 
        self.plot_patch_list = []
        self.plot_line_list = []
        self.plot_text_list = []

        self.plot_kwargs = kwargs.get('plot', dict())


    def __repr__(self) -> str:
        pass

    def __eq__(self, o: object) -> bool:
        return self._id == o._id


    @classmethod
    def create_with_shape(cls, kinematics_name, shape_dict, **kwargs):

        shape_name = shape_dict.get('name', 'circle')   
             
        if shape_name == 'circle':

            radius = shape_dict.get('radius', 0.2) 

            return cls(shape='circle', shape_tuple=(0, 0, radius), **kwargs)
 
        elif shape_name == 'rectangle':

            if kinematics_name == 'diff':
                length = shape_dict.get('length', 0.2)
                width = shape_dict.get('width', 0.1)

                return cls(shape='polygon', shape_tuple=[(-length/2, -width/2), (length/2, -width/2), (length/2, width/2), (-length/2, width/2)], length=length, width=width,**kwargs)
            
            elif kinematics_name == 'acker':

                length = shape_dict.get('length', 4.6)
                width = shape_dict.get('width', 1.6)
                wheelbase = shape_dict.get('wheelbase', 3)

                start_x = - (length - wheelbase)/2
                start_y = - width/2

                vertices = [(start_x, start_y), (start_x + length, start_y), (start_x+length, start_y+width), (start_x, start_y + width) ]

                return cls(shape='polygon', shape_tuple=vertices, wheelbase=wheelbase, length=length, width=width, **kwargs)
            
            else:
                length = shape_dict.get('length', 0.2)
                width = shape_dict.get('width', 0.1)

                return cls(shape='polygon', shape_tuple=[(-length/2, -width/2), (length/2, -width/2), (length/2, width/2), (-length/2, width/2)], **kwargs)

        elif shape_name == 'polygon':

            if shape_dict.get('random_shape', False):
                vertices = random_generate_polygon(**shape_dict)
            else:
                vertices = shape_dict.get('vertices', None)

            if vertices is None:
                raise ValueError("vertices are not set")

            return cls(shape='polygon', shape_tuple=vertices, **kwargs)

        elif shape_name == 'linestring':

            vertices = shape_dict.get('vertices', None)

            if vertices is None:
                raise ValueError("vertices should not be None")
            
            return cls(shape='linestring', shape_tuple=vertices, **kwargs)
        
        elif shape_name == 'points':
            pass

        else:
            raise NotImplementedError(f"shape {shape_name} not implemented")


    def step(self, velocity=None, **kwargs):


        if self.static or self.stop_flag:

            self._velocity = np.zeros_like(velocity)

            return self._state  

        else: 
            
            self.pre_process()

            behavior_vel = self.gen_behavior_vel(velocity, )

            new_state = self._kinematics(behavior_vel, **self.kinematics_dict, **kwargs)
            next_state = self.mid_process(new_state)

            self._state = next_state
            self._velocity = behavior_vel

            self._geometry = self.geometry_transform(self._init_geometry, self._state)

            self.sensor_step()
            self.post_process()
            self.check_status()

            self.trajectory.append(self._state.copy())
                
            return next_state


    def sensor_step(self):
        [sensor.step(self._state[0:3]) for sensor in self.sensors]
    

    def _kinematics(self, velocity, **kwargs):
        # default is omni
        if not self.static:
            new_state = self._state[0:2] + velocity * world_param.step_time
        else:
            new_state = self._state

        return new_state

    def geometry_transform(self, geometry, state):

        def transform_with_state(x, y):

            trans, rot = get_transform(state)
            points = np.array([x, y])

            new_points = rot @ points + trans

            return (new_points[0, :], new_points[1, :])
        
        new_geometry = transform(transform_with_state, geometry)

        return new_geometry

    
    # check arrive
    def check_status(self):
        
        self.check_arrive_status()
        self.check_collision_status()

        if world_param.collision_mode == 'stop':
            self.stop_flag = self.collision_flag
        elif world_param.collision_mode == 'reactive':
            pass

        elif world_param.collision_mode == 'unobstructed':
            pass

        
    def check_arrive_status(self):

        if self.arrive_mode == 'state':
            diff = np.linalg.norm(self._state[:3] - self._goal[:3])
        elif self.arrive_mode == 'position':
            diff = np.linalg.norm(self._state[:2] - self._goal[:2])

        if diff < self.goal_threshold:
            self.arrive_flag = True
        else:
            self.arrive_flag = False

    
    def check_collision_status(self):
        
        collision_flags = [ self.check_collision(obj) for obj in env_param.objects if self.id != obj.id]

        new_collision_flag = any(collision_flags)

        if new_collision_flag and not self.collision_flag:
            env_param.logger.warning( self.role + "{} is collided at state {}".format(self.id, list(np.round(self._state[:2, 0], 2))))
            
        self.collision_flag = new_collision_flag


    def check_collision(self, obj):
        return shapely.intersects(self._geometry, obj._geometry)

    
    def gen_behavior_vel(self, velocity):

        min_vel, max_vel = self.get_vel_range()
        
        if velocity is None:
            
            if self.obj_behavior.behavior_dict is None:
                # self.static = True

                if self.role=='robot': print("Warning: behavior and input velocity is not defined, robot will stay static")

                return np.zeros_like(self._velocity)

            else:
                
                if self.obj_behavior.behavior_dict['name'] == 'wander':
                    
                    if self.arrive_flag:
                        range_low = np.c_[self.obj_behavior.behavior_dict['range_low']]
                        range_high = np.c_[self.obj_behavior.behavior_dict['range_high']]

                        self._goal = np.random.uniform(range_low, range_high)
                        self.arrive_flag = False

                if self.obj_behavior.behavior_dict['name'] == 'rvo':
                    behavior_vel = self.obj_behavior.gen_vel(self._state, self._goal, min_vel, max_vel, rvo_neighbor = self.rvo_neighbors, rvo_state=self.rvo_state)
                
                else:
                    behavior_vel = self.obj_behavior.gen_vel(self._state, self._goal, min_vel, max_vel)

            
        else:
            if isinstance(velocity, list): velocity = np.c_[velocity]
            if velocity.ndim == 1: velocity = velocity[:, np.newaxis]

            assert velocity.shape == self.vel_dim

            behavior_vel = velocity

        # input_kwargs = {'state': self._state, 'goal': self._goal, 'min_vel': min_vel, 'max_vel': max_vel}

        if ( behavior_vel < (min_vel - 0.01) ).any():
            logging.warning("input velocity {} is smaller than min_vel {}, velocity is clipped".format(behavior_vel.flatten(), min_vel.flatten()))
        elif (behavior_vel > (max_vel + 0.01 )).any():
            logging.warning("input velocity {} is larger than max_vel {}, velocity is clipped".format(behavior_vel.flatten(), max_vel.flatten()))

        behavior_vel_clip = np.clip(behavior_vel, min_vel, max_vel)


        return behavior_vel_clip



    def custom_behavior(self, velocity, min_vel, max_vel):
        pass


    def vel_check(self, velocity):
        
        if isinstance(vel, list): vel = np.c_[vel]
        if velocity.ndim == 1: vel = vel[:, np.newaxis]

        assert velocity.shape == self.vel_dim
 
        min_vel = np.maximum(self.vel_min, self._velocity - self.acce * world_param.step_time)
        max_vel = np.minimum(self.vel_max, self._velocity + self.acce * world_param.step_time)

        if (velocity < min_vel).any():
            logging.warning("velocity is smaller than min_vel, velocity is clipped to min_vel")
        elif (velocity > max_vel).any():
            logging.warning("velocity is larger than max_vel, velocity is clipped to max_vel")

        velocity_clip = np.clip(velocity, min_vel, max_vel)

        return velocity_clip

    def pre_process(self):
        # collision check
        pass
        

    def post_process(self):
        pass
    
    def mid_process(self, state):
        
        if state.shape[0] > 2:
            state[2, 0] = WrapToRegion(state[2, 0], self.info.angle_range)

        return state


    def get_lidar_scan(self):
        return self.lidar.get_scan()

    def get_lidar_points(self):
        return self.lidar.get_points()




    def construct_geometry(self, shape, shape_tuple, reso=0.1):

        if shape == 'circle':
            geometry = Point([ shape_tuple[0], shape_tuple[1] ]).buffer(shape_tuple[2])

        elif shape == 'polygon' or shape =='rectangle':
            geometry = Polygon(shape_tuple)

        elif shape == 'linestring':
            geometry = LineString(shape_tuple)

        elif shape == 'points':
            geometry = MultiPoint(shape_tuple.T).buffer(reso/2)

        else:
            raise ValueError("shape should be one of the following: circle, polygon, linestring")

        return geometry


    def geometry_state_transition(self):
        pass
    
    
    def plot(self, ax, **kwargs):
        
        self.state_re = self._state
        self.goal_re = self._goal

        self.plot_kwargs.update(kwargs)

        show_goal = self.plot_kwargs.get('show_goal', False) 
        show_text = self.plot_kwargs.get('show_text', False)
        show_arrow = self.plot_kwargs.get('show_arrow', False)
        show_uncertainty = self.plot_kwargs.get('show_uncertainty', False)
        show_trajectory = self.plot_kwargs.get('show_trajectory', False)
        show_trail = self.plot_kwargs.get('show_trail', False)
        show_sensor = self.plot_kwargs.get('show_sensor', True)
        trail_freq = self.plot_kwargs.get('trail_freq', 2)
        goal_color = self.plot_kwargs.get('goal_color', self.color)

        self.plot_object(ax, **kwargs)

        if show_goal:
            self.plot_goal(ax, goal_color)

        if show_text:
            self.plot_text(ax)

        if show_arrow:
            self.plot_arrow(ax, **self.plot_kwargs)

        if show_uncertainty:
            self.plot_uncertainty(ax, **self.plot_kwargs)

        if show_trajectory:
            self.plot_trajectory(ax, **self.plot_kwargs)
        
        if show_trail and world_param.count % trail_freq == 0:
            self.plot_trail(ax, **self.plot_kwargs)

        if show_sensor:
            [sensor.plot(ax, **self.plot_kwargs) for sensor in self.sensors]

        
    def plot_object(self, ax, **kwargs):

        if self.description is None:

            x = self.state_re[0, 0]
            y = self.state_re[1, 0]

            if self.shape == 'circle':

                object_patch = mpl.patches.Circle(xy=(x, y), radius = self.radius, color = self.color)
                object_patch.set_zorder(3)

                ax.add_patch(object_patch)

            elif self.shape == 'polygon':
                object_patch = mpl.patches.Polygon(xy=self.vertices.T, color=self.color)
                object_patch.set_zorder(3)

                ax.add_patch(object_patch)
                

            elif self.shape == 'linestring':
                object_patch = mpl.lines.Line2D(self.vertices[0, :], self.vertices[1, :], color=self.color)
                object_patch.set_zorder(3)
                ax.add_line(object_patch)
                # line.remove()
            
            elif self.shape == 'points':
                return 
            
            self.plot_patch_list.append(object_patch)
            # self.plot_patch_list.append(object_patch)

        else:
            self.plot_object_image(ax, self.description, **kwargs)



    def plot_object_image(self):
        pass
    

    def plot_trajectory(self, ax, keep_length=0, **kwargs):
        
        traj_color = kwargs.get('traj_color', self.color)
        traj_style = kwargs.get('traj_style', '-')

        # x_list = [t[0, 0] for t in self.trajectory]
        # y_list = [t[1, 0] for t in self.trajectory]
        x_list = [t[0, 0] for t in self.trajectory[-keep_length:]]
        y_list = [t[1, 0] for t in self.trajectory[-keep_length:]]
        
        self.plot_line_list.append(ax.plot(x_list, y_list, color=traj_color, linestyle=traj_style))
        

    def plot_goal(self, ax, goal_color='r'):

        goal_x = self.goal_re[0, 0]
        goal_y = self.goal_re[1, 0]
        
        goal_circle = mpl.patches.Circle(xy=(goal_x, goal_y), radius=self.radius, color=goal_color, alpha=0.5)
        goal_circle.set_zorder(1)
    
        ax.add_patch(goal_circle)

        self.plot_patch_list.append(goal_circle)


    def plot_text(self, ax, **kwargs):
        pass

    def plot_arrow(self, ax, arrow_length=0.4, arrow_width=0.6, **kwargs):

        x = self.state_re[0][0]
        y = self.state_re[1][0]
        theta = self.state_re[2][0]
        arrow_color = kwargs.get('arrow_color', self.color)

        arrow = mpl.patches.Arrow(x, y, arrow_length*cos(theta), arrow_length*sin(theta), width=arrow_width, color=arrow_color)
        arrow.set_zorder(3)
        ax.add_patch(arrow)
        
        self.plot_patch_list.append(arrow)

    def plot_trail(self, ax, **kwargs):
        
        trail_type = kwargs.get('trail_type', self.shape)
        trail_edgecolor = kwargs.get('edgecolor', self.color)
        trail_linewidth = kwargs.get('linewidth', 0.8)
        trail_alpha = kwargs.get('trail_alpha', 0.7)
        trail_fill = kwargs.get('trail_fill', False)
        trail_color = kwargs.get('trail_color', self.color)

        r_phi_ang = 180 * self._state[2, 0] / pi
        

        if trail_type == 'rectangle' or trail_type == 'polygon':

            start_x = self.vertices[0, 0]
            start_y = self.vertices[1, 0]

            car_rect = mpl.patches.Rectangle(xy=(start_x, start_y), width=self.length, height=self.width, angle=r_phi_ang, edgecolor=trail_edgecolor, fill=False, alpha=trail_alpha, linewidth=trail_linewidth, facecolor=trail_color)
            ax.add_patch(car_rect)

        elif trail_type == 'circle':
            
            car_circle = mpl.patches.Circle(xy=self.centroid, radius = self.radius, edgecolor=trail_edgecolor, fill=trail_fill, alpha=trail_alpha, facecolor=trail_color)
            ax.add_patch(car_circle)


    def plot_uncertainty(self, ax, **kwargs):
        pass

    def plot_clear(self):
        
        [patch.remove() for patch in self.plot_patch_list]
        [line.pop(0).remove() for line in self.plot_line_list]
        [text.remove() for text in self.plot_text_list]

        self.plot_patch_list = []
        self.plot_line_list = []
        self.plot_text_list = []

        [sensor.plot_clear() for sensor in self.sensors]


    def done(self):

        if self.stop_flag or self.arrive_flag:
            return True
        else:
            return False
        
    def reset(self):
        self._state = self._init_state.copy()
        self._goal = self._init_goal.copy()
        self._velocity = self._init_velocity.copy()

        self.collision_flag = False
        self.arrive_flag = False
        self.stop_flag = False
        self.trajectory = []



    # generate random polygon
    @staticmethod
    def random_generate_polygons(number, avg_radius, irregularity, spikiness, num_vertices):   
        pass


    # get information

    # def get_inequality_Ab(self, s):
    #     # general inequality Ax <= b 
        
    #     if self.shape == 'circle':
    #         pass
            
    #     elif self.shape == 'polygon':
    #         pass

    #     elif self.shape == 'linestring':
    #         pass
        
    #     return A, b

    def get_vel_range(self):

        min_vel = np.maximum(self.vel_min, self._velocity - self.info.acce * world_param.step_time)
        max_vel = np.minimum(self.vel_max, self._velocity + self.info.acce * world_param.step_time)

        return min_vel, max_vel


    def set_state(self, state):

        if isinstance(state, list): state = np.c_[state]

        state[2, 0] = WrapToRegion(state[2, 0], self.info.angle_range)

        self._state = state
    

    # get information
    def get_info(self):
        return self.info

    # property
    @property
    def name(self):
        return self.info.role + '_' + str(self.id)

    @property
    def shape(self):
        return self._shape

    @property
    def geometry(self):
        return self._geometry

    @property
    def centroid(self):
        return self._geometry.centroid.coords._coords.T
        
    @property
    def id(self):
        return self._id
    
    @property
    def state(self):
        return self._state

    @property
    def goal(self):
        return self._goal
    
    @property
    def position(self):
        # return the position of the object
        return self._state[:2]

    @property
    def radius(self):

        '''
        return the minimum bounding radius
        '''
    
        return minimum_bounding_radius(self._geometry)
    
    @property
    def radius_extend(self):
        return self.radius + 0.1

    @property
    def arrive(self):
        return self.arrive_flag
    
    @property
    def collision(self):
        return self.collision_flag

    @property
    def ineq_Ab(self):
        return self.get_inequality_Ab()
    
    @property
    def vertices(self):
        # 2*N

        if self.shape == 'linestring':
            x = self._geometry.xy[0]
            y = self._geometry.xy[1]

            return np.c_[x, y].T

        return self._geometry.exterior.coords._coords.T

    @property
    def desired_diff_vel(self, angle_tolerance=0.1, goal_threshold=0.1):

        
        distance, radian = relative_position(self._state, self._goal) 

        if distance < goal_threshold:
            return np.zeros((2, 1))

        diff_radian = WrapToPi( radian - self._state[2, 0] )

        linear = self.vel_max[0, 0] * np.cos(diff_radian)

        if abs(diff_radian) < angle_tolerance:
            angular = 0
        else:
            angular = self.vel_max[1, 0] * np.sign(diff_radian)

        return np.array([[linear], [angular]])

    @property
    def desired_omni_vel(self, goal_threshold=0.1):
         
        dis, radian = relative_position(self.state, self.goal)    

        if dis > goal_threshold:
            vx = self.vel_max[0, 0] * cos(radian)
            vy = self.vel_max[1, 0] * sin(radian)
        else:
            vx = 0
            vy = 0

        return np.array([[vx], [vy]])
    


    @property
    def rvo_neighbors(self):
        # a list of rvo neighbors [x, y, vx, vy, radius]
        return [ obj.rvo_neighbor_state for obj in env_param.objects if self.id != obj.id]

    @property
    def rvo_neighbor_state(self):
        # x, y, vx, vy, radius
        return [self._state[0, 0], self._state[1, 0], self.velocity_xy[0, 0], self.velocity_xy[1, 0], self.radius_extend]
    
    @property
    def rvo_state(self):
        vx_des, vy_des = self.desired_omni_vel[:, 0]
        # x, y, vx, vy, radius, vx_des, vy_des, theta
        return [self._state[0, 0], self._state[1, 0], self.velocity_xy[0, 0], self.velocity_xy[1, 0], self.radius_extend, vx_des, vy_des, self._state[2, 0]]


        

