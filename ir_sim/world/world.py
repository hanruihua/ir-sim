import itertools
from ir_sim.util.util import file_check
from ir_sim.global_param import world_param
import numpy as np
import matplotlib.image as mpimg
from skimage.color import rgb2gray

class world:
    def __init__(self, name='world', height=10, width=10, step_time=0.1, sample_time=0.1, offset=[0, 0], control_mode='auto', collision_mode='stop',  obstacle_map=None) -> None:
        
        '''
        the world object is the main object of the simulation, it manages all the other objects and maps in the simulation

        Parameters:
            height: the height of the world
            width: the width of the world
            step_time: the time interval between two steps
            sample_time: the time interval between two samples

            collision_mode: the collision mode of the world, 
                            'stop'
                            'reactive'
                            'unobstructed'

            control_mode: the control mode of the world,
                            'auto'
                            'keyboard'
        '''

        self.name = name
        self.height = height
        self.width = width
        self.step_time = step_time
        self.sample_time = sample_time
        self.offset = offset

        self.count = 0
        self.sampling = True

        self.x_range = [self.offset[0], self.offset[0] + self.width]
        self.y_range = [self.offset[1], self.offset[1] + self.height]

        self.grid_map, self.obstacle_index, self.obstacle_positions = self.gen_grid_map(obstacle_map)


        # set world param
        world_param.step_time = step_time
        world_param.control_mode = control_mode
        world_param.collision_mode = collision_mode

    
    def step(self):

        self.count += 1
        self.sampling = (self.count % (self.sample_time / self.step_time) == 0)

        world_param.time = self.time
        world_param.count = self.count


    def gen_grid_map(self, obstacle_map):

        abs_obstacle_map = file_check(obstacle_map)

        # px = int(self.width / self.xy_reso)
        # py = int(self.height / self.xy_reso)
        if abs_obstacle_map is not None:

            grid_map = mpimg.imread(abs_obstacle_map)

            if len(grid_map.shape) > 2:
                grid_map = rgb2gray(grid_map)  
            
            grid_map = 100 * (1 - grid_map)   # range: 0 - 100
            grid_map = np.fliplr(grid_map.T)
         
            x_reso = self.width / grid_map.shape[0]
            y_reso = self.height / grid_map.shape[1]
            self.reso = np.array([[x_reso], [y_reso]])

            obstacle_index = np.array(np.where(grid_map > 50))
            obstacle_positions = obstacle_index * self.reso

        else:
            grid_map = None
            obstacle_index = None
            obstacle_positions = None
            self.reso = np.zeros((2, 1))

        return grid_map, obstacle_index, obstacle_positions


    @property
    def time(self):
        return self.count * self.step_time
    
    @property
    def buffer_reso(self):
        return np.max(self.reso)
    

    


    

