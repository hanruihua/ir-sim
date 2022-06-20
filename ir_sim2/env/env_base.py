import logging
import sys
import yaml
import numpy as np
import matplotlib.pyplot as plt

from PIL import Image
from pynput import keyboard
from .env_robot import EnvRobot
from .env_obstacle import EnvObstacle
from ir_sim2.world import RobotDiff, RobotAcker, RobotOmni
from ir_sim2.log.Logger import Logger

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
                obstacle_args_list = com_list.get('obstacles', [dict()])

        world_args.update(kwargs.get('world_args', dict()))
        robot_args.update(kwargs.get('robot_args', dict()))
        [obstacle_args.update(kwargs.get('obstacle_args', dict())) for obstacle_args in obstacle_args_list]

        # world args
        self.__height = world_args.get('world_height', 10)
        self.__width = world_args.get('world_width', 10) 
        self.step_time = world_args.get('step_time', 0.1) 
        self.offset_x = world_args.get('offset_x', 0) 
        self.offset_y = world_args.get('offset_y', 0)

        self.robot_args = robot_args
        self.obstacle_args_list = obstacle_args_list

        self.plot = plot
        self.components = dict()
        self.init_environment(**kwargs)

        self.log = Logger('robot.log', level='info')

        if control_mode == 'keyboard':
            pass

    def init_environment(self, **kwargs):
        # full=False, keep_path=False, 
        # kwargs: full: False,  full windows plot
        #         keep_path, keep a residual
        #         robot kwargs:
        #         obstacle kwargs:
        self.env_robot = EnvRobot(EnvBase.robot_factory[self.robot_args['type']], step_time=self.step_time, **self.robot_args)
        self.env_obstacle_list = [ EnvObstacle()]
        # self.env_robot_list = [EnvRobot(EnvBase.robot_factory[ra['type']], step_time=self.step_time, **ra) for ra in self.robot_args_list]

        # default robots 
        self.robot_list = self.env_robot.robot_list
        self.robot = self.robot_list[0]

        # plot
        if self.plot:
            self.fig, self.ax = plt.subplots()
            self.init_plot(self.ax, **kwargs)
            # self.fig, self.ax = plt.subplots()
        
        # self.components['env_robot_list'] = self.env_robot_list
        self.components['env_robot'] = self.env_robot

    def cal_des_vel(self, **kwargs):
        return [robot.cal_des_vel(**kwargs) for robot in self.robot_list]
        
    def robots_step(self, vel_list, **kwargs):
        self.env_robot.move(vel_list, **kwargs)

    def step(self, vel_list, **kwargs):
        self.robots_step(vel_list, **kwargs)

    def done(self):
        if self.env_robot.arrive():
            self.log.logger.info('All robots arrive at the goal positions')
        if self.env_robot.collision():
            self.log.logger.warning('Collisions Occur')
        return self.env_robot.arrive()
    
    def done_list(self):
        return self.env_robot.arrive_list()

    def render(self, time=0.0001, **kwargs):

        if self.plot:
            self.draw_components(self.ax, mode='dynamic', **kwargs)
            plt.pause(time)
            self.clear_components(self.ax, mode='dynamic', **kwargs)
            
    def init_plot(self, ax, **kwargs):
        ax.set_aspect('equal')
        ax.set_xlim(self.offset_x, self.offset_x + self.__width)
        ax.set_ylim(self.offset_y, self.offset_y + self.__height)
        
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")

        self.draw_components(ax, mode='static', **kwargs)
    
    def draw_components(self, ax, mode='all', **kwargs):
        # mode: static, dynamic, all
        if mode == 'static':
            pass
        elif mode == 'dynamic':
            self.env_robot.plot(ax, **kwargs)
            # obstacle
        elif mode == 'all':
            self.env_robot.plot(ax, **kwargs)
            # obstacle
        else:
            self.logger.error('error input of the draw mode')

    def clear_components(self, ax, mode='all', **kwargs):
        if mode == 'dynamic':
            self.env_robot.plot_clear(ax)
        elif mode == 'static':
            pass
        elif mode == 'all':
            plt.cla()
        

    def show(self, **kwargs):
        self.draw_components(self.ax, mode='dynamic', **kwargs)
        plt.show()



            



    


            