import numpy as np
from ir_sim.world.robots.robot_diff import RobotDiff
from ir_sim.world.robots.robot_acker import RobotAcker
from ir_sim.world.robots.robot_omni import RobotOmni
from ir_sim.world import ObjectBase
from ir_sim.world.obstacles.obstacle_diff import ObstacleDiff
from ir_sim.world.obstacles.obstacle_omni import ObstacleOmni
from ir_sim.world.obstacles.obstacle_static import ObstacleStatic
from ir_sim.world.map.obstacle_map import ObstacleMap
from ir_sim.util.util import convert_list_length, convert_list_length_dict, is_list_of_numbers
from ir_sim.global_param import env_param 
import random

class ObjectFactory:
    
    
    def create_from_parse(self, parse, obj_type='robot'):
        # create object from yaml file parse

        object_list = list()

        if isinstance(parse, list):
            object_list = [obj for sp in parse for obj in self.create_object(obj_type, **sp)]

        elif isinstance(parse, dict):
            object_list = [obj for obj in self.create_object(obj_type, **parse)]

        return object_list


    def create_from_map(self, points, reso=0.1):

        if points is None:
            return []
        else:
            return [ObstacleMap(shape='points', shape_tuple=points, color='k', reso=reso)]

        

    def create_object(self, obj_type='robot', number=1, distribution={'name': 'manual'}, state=[1, 1, 0], goal=[1, 9, 0], **kwargs):

        '''
        create object based on the number of objects to create, object type, distribution of states and initial state
            - obj_type: 'robot' or 'obstacle'
            - number: number of objects to create
            - distribution: distribution of states for objects
            - state: initial state for objects
            - kwargs: other parameters for object creation
        '''
        
        state_list, goal_list = self.generate_state_list(number, obj_type, distribution, state, goal)
        object_list = list()

        for i in range(number):
            obj_dict = dict()
            
            obj_dict = {k: convert_list_length(v, number)[i] for k, v in kwargs.items() if k != 'sensors'}
            obj_dict['state'] = state_list[i]
            obj_dict['goal'] = goal_list[i]
            obj_dict['sensors'] = convert_list_length_dict(kwargs.get('sensors', None), number)[i]

            if obj_type == 'robot':
                object_list.append(self.create_robot(**obj_dict))
            elif obj_type == 'obstacle':
                object_list.append(self.create_obstacle(**obj_dict))

        return object_list
    
    
    def create_robot(self, kinematics=dict(), shape=dict(), **kwargs):

        # kinematics_name = kinematics.pop('name', 'omni')
        kinematics_name = kinematics.get('name', 'omni')
        
        if kinematics_name == 'diff':
            return RobotDiff.create_with_shape('diff', shape, kinematics_dict=kinematics, **kwargs)
        elif kinematics_name == 'acker':
            return RobotAcker.create_with_shape('acker', shape, kinematics_dict=kinematics, **kwargs)
        elif kinematics_name == 'omni':
            return RobotOmni.create_with_shape('omni', shape, kinematics_dict=kinematics, **kwargs)
        else:
            raise NotImplementedError(f"Robot kinematics {kinematics_name} not implemented")


    def create_obstacle(self, kinematics=dict(), shape=dict(), **kwargs):
        
        kinematics_name = kinematics.get('name', None)
  
        if kinematics_name == 'diff':
            return ObstacleDiff.create_with_shape(kinematics_name, shape, kinematics_dict=kinematics, **kwargs)
        elif kinematics_name == 'acker':
            pass
        elif kinematics_name == 'omni':
            return ObstacleOmni.create_with_shape(kinematics_name, shape, kinematics_dict=kinematics, **kwargs)
        else:
            return ObstacleStatic.create_with_shape(kinematics_name, shape, kinematics_dict=kinematics, **kwargs)


    def generate_state_list(self, number=1, obj_type='robot', distribution={'name': 'manual'}, state=[1, 1, 0], goal=[1, 9, 0]):

        '''
        Generate state list for robots or obstacles based on distribution and state provided in kwargs
            - number: number of objects to generate
            - obj_type: 'robot' or 'obstacle'
            - distribution: distribution dictionary of states for objects
            - state: initial state for objects
        '''
        
        if distribution['name'] == 'manual':
            state_list = convert_list_length(state, number)
            goal_list = convert_list_length(goal, number)


        elif distribution['name'] == 'random':
            
            range_low = distribution.get('range_low', [0, 0, -np.pi])
            range_high = distribution.get('range_high', [10, 10, np.pi])

            state_array = np.random.uniform(low=range_low, high=range_high, size=(number, 3))
            state_list = state_array.tolist()

            goal_array = np.random.uniform(low=range_low, high=range_high, size=(number, 3))
            goal_list = goal_array.tolist()

        
        elif distribution['name'] == 'uniform':
            pass
        
        elif distribution['name'] == 'circle':
            
            radius = distribution.get('radius', 4)
            center = distribution.get('center', [5, 5, 0])

            state_list, goal_list = [], []
            for i in range(number):
                theta = 2*np.pi*i/number
                x = center[0] + radius*np.cos(theta)
                y = center[1] + radius*np.sin(theta)
                state_list.append([x, y, theta-np.pi])

                goal_x = center[0] - radius*np.cos(theta)
                goal_y = center[1] - radius*np.sin(theta)
                goal_list.append([goal_x, goal_y, 0])

        return state_list, goal_list
            
        #     x_range = distribution.get('x_range', (0, 10))
        #     y_range = distribution.get('y_range', (0, 10))
        #     theta_range = distribution.get('theta_range', (0, 2*np.pi))

        #     state_list = []
        #     for _ in range(number):
        #         x = random.uniform(*x_range)
        #         y = random.uniform(*y_range)
        #         theta = random.uniform(*theta_range)
        #         state_list.append([x, y, theta])
                 




