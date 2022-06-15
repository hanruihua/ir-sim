import sys
import yaml
import numpy as np
import matplotlib.pyplot as plt

from PIL import Image
from pynput import keyboard
from .env_robot import EnvRobot
from ir_sim2.world import RobotDiff, RobotAcker, RobotOmni

class EnvBase:

    robot_factory={'robot_diff': RobotDiff, 'robot_acker': RobotAcker, 'robot_omni': RobotOmni}

    def __init__(self, world_name=None, plot=True, control_mode='auto', **kwargs):
        
        # world_name: path of the yaml
        # plot: True or False
        # control_mode: auto, keyboard
        # 
        world_args, robot_args, obstacle_args = dict(), dict(), dict() 

        if world_name != None:
            world_name = sys.path[0] + '/' + world_name

            with open(world_name) as file:
                com_list = yaml.load(file, Loader=yaml.FullLoader)
                world_args = com_list.get('world', dict())
                robot_args = com_list.get('robots', dict())
                obstacle_args = com_list.get('obstacles', dict())

        world_args.update(kwargs.get('world_args', dict()))
        robot_args.update(kwargs.get('robot_args', dict()))
        obstacle_args.update(kwargs.get('obstacle_args', dict()))

        # world args
        self.__height = world_args.get('world_height', 10)
        self.__width = world_args.get('world_width', 10) 
        self.step_time = world_args.get('step_time', 0.1) 
        self.offset_x = world_args.get('offset_x', 0) 
        self.offset_y = world_args.get('offset_y', 0)

        self.robot_args = robot_args
        self.obstacle_args = obstacle_args

        self.plot = plot
        self.components = dict()
        self.init_environment(**kwargs)

        if control_mode == 'keyboard':
            pass

    def init_environment(self, **kwargs):
        # full=False, keep_path=False, 
        # kwargs: full: False,  full windows plot
        #         keep_path, keep a residual
        #         robot kwargs:
        #         obstacle kwargs:
        self.robot_group = EnvRobot(EnvBase.robot_factory[self.robot_args['type']], step_time=self.step_time, **self.robot_args)
        self.robot = self.robot_group.robot_list[0]

        # plot
        if self.plot:
            self.init_plot(**kwargs)
            # self.fig, self.ax = plt.subplots()
        
        self.components['robot_group'] = self.robot_group

    
    def robots_step(self, vel_list):
        self.robot_group.move(vel_list)
        
    def render(self, time=0.0001, **kwargs):

        if self.plot:
            self.robot_group.plot(self.ax, **kwargs) 
            plt.pause(time)
            self.robot_group.plot_clear(self.ax)


    def init_plot(self, **kwargs):
        self.fig, self.ax = plt.subplots()

        self.ax.set_aspect('equal')
        self.ax.set_xlim(self.offset_x, self.offset_x + self.__width)
        self.ax.set_ylim(self.offset_y, self.offset_y + self.__height)
        
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")

        # self.draw_components(**kwargs)    
    
    def draw_components(self, **kwargs):
        # for 
        pass
    

    def show(self, **kwargs):
        plt.show()



            



    


            