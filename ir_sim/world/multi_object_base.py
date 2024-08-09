import numpy as np
from ir_sim.util.util import extend_list
from ir_sim.world.object_base import ObjectBase
from ir_sim.lib.generation import generate_polygon

class MultiObjects:
    def __init__(self, kinematics, number, distribution, **kwargs) -> None:
        
        self.number = number
        self.kinematics = kinematics

        # self.object_list = [object_class(**kwargs) for _ in range(number)]
        self.state_list, self.shape_list = self.generate_state_shape(distribution, **kwargs)

        behavior_list = kwargs.get('behaviors', [])
        self.behavior_list = extend_list(behavior_list, self.number)

        if self.behavior_list is None:
            self.object_list = [ ObjectBase.create_with_shape(kinematics, shape, state=state, **kwargs) for state, shape in zip(self.state_list, self.shape_list) ]

        else:
            self.object_list = [ ObjectBase.create_with_shape(kinematics, shape, state=state, behavior=behavior, **kwargs) for state, shape, behavior in zip(self.state_list, self.shape_list, self.behavior_list) ]


    def __add__(self, other):
        return self.object_list + other.object_list

    def __len__(self):
        return len(self.object_list)

    def __getitem__(self, index):
        return self.object_list[index]
    
    
    def generate_state_shape(self, distribution_dict, **kwargs):


        if distribution_dict is None:
            # default
            state_list = kwargs['states']
            
        elif distribution_dict['mode'] == 'manual':
            # state_list = kwargs['states']
            state_list = kwargs.get('states', [[0, 0, 0]] * self.number)


        elif distribution_dict['mode'] == 'random':
            range_low = distribution_dict.get('range_low', [0, 0, -np.pi])
            range_high = distribution_dict.get('range_high', [10, 10, np.pi])

            state_array = np.random.uniform(low=range_low, high=range_high, size=(self.number, 3))
            state_list = state_array.tolist()
           
        shape_list = kwargs['shapes']

        if distribution_dict.get('random_bear', False):
            pass

        state_list = extend_list(state_list, self.number)
        shape_list = extend_list(shape_list, self.number)
        
        return state_list, shape_list



    def step(self, velocity_list=[]):

        for obj, vel in  zip(self.object_list, velocity_list):
            obj.step(vel)


    def plot(self, ax, **kwargs):

        for obj in self.object_list:
            obj.plot(ax, **kwargs)



    def random_generate_polygon(self, number=1, center_range=[0, 0, 10, 10], avg_radius_range=[0.1, 1], irregularity_range=[0, 1], spikiness_range=[0, 1], num_vertices_range=[4, 10], **kwargs):

        ''' 
        2d range: min_x, min_y, max_x, max_y
        '''
        
        center = np.random.uniform(low=center_range[0:2], high=center_range[2:], size=(number, 2))
        avg_radius = np.random.uniform(low=avg_radius_range[0], high=avg_radius_range[1], size=(number,))
        irregularity = np.random.uniform(low=irregularity_range[0], high=irregularity_range[1], size=(number,))
        spikiness = np.random.uniform(low=spikiness_range[0], high=spikiness_range[1], size=(number,))
        num_vertices = np.random.randint(low=num_vertices_range[0], high=num_vertices_range[1], size=(number,))

        vertices_list = [generate_polygon(center[i, :], avg_radius[i], irregularity[i], spikiness[i], num_vertices[i]) for i in range(number)]
        
        return vertices_list





       
    
        



        
        
        
    





    