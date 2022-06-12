import sys
import yaml
import numpy as np
import matplotlib.pyplot as plt

from PIL import Image
from pynput import keyboard
from ir_sim2.env import EnvPlot
from ir_sim2.env import EnvRobot
from ir_sim2.world import RobotDiff, RobotAcker, RobotOmni

class EnvBase:

    robot_factory={'robot_diff': RobotDiff, 'robot_acker': RobotAcker, 'robot_omni': RobotOmni}

    def __init__(self, world_name=None, plot=True, control_mode='auto', **kwargs):

        # plot
        # control_mode: auto, keyboard

        if world_name != None:
            world_name = sys.path[0] + '/' + world_name

            with open(world_name) as file:
                com_list = yaml.load(file, Loader=yaml.FullLoader)
                world_args = com_list.get('world', dict())

                self.__height = world_args.get('world_height', 10)
                self.__width = world_args.get('world_width', 10)
                self.step_time = world_args.get('step_time', 0.1)
                self.offset_x = world_args.get('offset_x', 0)
                self.offset_y = world_args.get('offset_y', 0)

                map_args = com_list.get('map', dict())
                self.image = map_args.get('map_image', None)
                self.reso = map_args.get('map_reso', 0.1)  

                self.robots_args = com_list.get('robot_group', dict())
                self.obstacles_args = com_list.get('obstacle_group', dict())

        self.plot = plot
        self.components = dict()
        self.init_environment(**kwargs)

    def init_environment(self, robot_type='diff', robot_shape='circle', **kwargs):
        # full=False, keep_path=False, 
        # kwargs: full: False,  full windows plot
        #         keep_path, keep a residual
        #         robot kwargs:
        #         obstacle kwargs:

        # self.robot_group = 
        # robot_class, robot_number=1, distribute_mode='manual', step_time=0.1, **kwargs)
        robot_class = EnvBase.robot_factory[robot_type]
        self.components['robots'] = EnvRobot(robot_class=robot_class, step_time=self.step_time, **kwargs)
        self.robot = self.components['robots'].robot_list[0]
        
        


        # plot
        if self.plot:
            self.fig, self.ax = plt.subplots()
            self.init_plot(**kwargs)

    def step(self):
        self.components['robots'].move()

    def render(self):
        pass


    def init_plot(self, **kwargs):

        self.ax.set_aspect('equal')
        self.ax.set_xlim(self.offset_x, self.offset_x + self.width)
        self.ax.set_ylim(self.offset_y, self.offset_y + self.height)
        
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")

        self.draw_components(**kwargs)    
    
    def draw_components(self):
        # for 
        pass
    

    def show(self, **kwargs):
        pass



            



    


            