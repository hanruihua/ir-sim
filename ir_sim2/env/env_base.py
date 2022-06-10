import sys
import yaml
import numpy as np
import matplotlib.pyplot as plt

from PIL import Image
from pynput import keyboard
from env_plot import EnvPlot

class EnvBase:
    def __init__(self, world_name=None, plot=True, control_mode='auto', **kwargs):

        # plot
        # control_mode: auto, keyboard

        if world_name != None:
            world_name = sys.path[0] + '/' + world_name

            with open(world_name) as file:
                com_list = yaml.load(file, Loader=yaml.FullLoader)
                world_args = com_list['world']

                self.__height = world_args.get('world_height', 10)
                self.__width = world_args.get('world_width', 10)
                self.step_time = world_args.get('step_time', 0.1)

                map_args = com_list['map']
                self.offset_x = map_args.get('offset_x', 0)
                self.offset_y = map_args.get('offset_y', 0)
                self.image = map_args.get('image', None)

                self.robots_args = com_list.get('robots', dict())

                self.obstacles_args = com_list.get('robots', dict())

        self.plot = plot
        self.components = dict()
        self.init_environment(**kwargs)

    def init_environment(self):
        
        # if self.plot:

            



    


            