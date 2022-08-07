import logging
import sys
import yaml
import imageio
import shutil
import numpy as np
import matplotlib.pyplot as plt

from pathlib import Path
from math import sin, cos
from PIL import Image
from pynput import keyboard
from .env_robot import EnvRobot
from .env_obstacle import EnvObstacle
from ir_sim2.world import RobotDiff, RobotAcker, RobotOmni, ObstacleCircle, ObstaclePolygon

class EnvBase:

    robot_factory={'robot_diff': RobotDiff, 'robot_acker': RobotAcker, 'robot_omni': RobotOmni}
    obstacle_factory = {'obstacle_circle': ObstacleCircle, 'obstacle_polygon': ObstaclePolygon}

    def __init__(self, world_name=None, plot=True, control_mode='auto', save_ani=False, **kwargs) -> None:
        
        # world_name: path of the yaml
        # plot: True or False
        # control_mode: auto, keyboard

        world_args, robot_args = dict(), dict()

        if world_name != None:
            world_name = sys.path[0] + '/' + world_name

            with open(world_name) as file:
                com_list = yaml.load(file, Loader=yaml.FullLoader)
                world_args = com_list.get('world', dict())
                robot_args = com_list.get('robots', dict())
                obstacle_args_list = com_list.get('obstacles', [])

        world_args.update(kwargs.get('world_args', dict()))
        robot_args.update(kwargs.get('robot_args', dict()))
        [obstacle_args.update(kwargs.get('obstacle_args', dict())) for obstacle_args in obstacle_args_list]

        # world, robot, obstacle, args
        self.__height = world_args.get('world_height', 10)
        self.__width = world_args.get('world_width', 10) 
        self.step_time = world_args.get('step_time', 0.01) 
        self.sample_time = world_args.get('sample_time', 0.1)
        self.offset_x = world_args.get('offset_x', 0) 
        self.offset_y = world_args.get('offset_y', 0)

        self.robot_args = robot_args
        self.obstacle_args_list = obstacle_args_list

        # plot
        self.plot = plot
        self.dyna_line_list = []

        # components
        self.components = dict()
        self.init_environment(**kwargs)

        self.count = 0
        self.sampling = True

        self.save_ani = save_ani  
        self.image_path = Path(sys.path[0] + '/' + 'image')  
        self.ani_path = Path(sys.path[0] + '/' + 'animation')
        
        if control_mode == 'keyboard':
            pass

    def init_environment(self, **kwargs):
        # full=False, keep_path=False, 
        # kwargs: full: False,  full windows plot
        #         keep_path, keep a residual
        #         robot kwargs:
        #         obstacle kwargs:
        if self.robot_args:
            self.env_robot = EnvRobot(self.robot_factory[self.robot_args['type']], step_time=self.step_time, **self.robot_args)
        else:
            self.env_robot = EnvRobot(self.robot_factory['robot_diff'], step_time=self.step_time, **self.robot_args)

        self.env_obstacle_list = [EnvObstacle(self.obstacle_factory[oa['type']], step_time=self.step_time, **oa) for oa in self.obstacle_args_list]
       
        # default robots 
        self.robot_list = self.env_robot.robot_list
        self.robot = self.robot_list[0] if len(self.robot_list) > 0 else None
        
        # default obstacles
        # plot
        if self.plot:
            self.fig, self.ax = plt.subplots()
            self.init_plot(self.ax, **kwargs)
            # self.fig, self.ax = plt.subplots()
        
        # self.components['env_robot_list'] = self.env_robot_list
        self.components['env_robot'] = self.env_robot

    def cal_des_vel(self, **kwargs):
        return self.env_robot.cal_des_vel(**kwargs)
         
    def robots_step(self, vel_list, **kwargs):
        self.env_robot.move(vel_list, **kwargs)

    def obstacles_step(self, **kwargs):
        [ env_obs.move() for env_obs in self.env_obstacle_list if env_obs.dynamic]

    def step(self, vel_list=[], **kwargs):
        self.robots_step(vel_list, **kwargs)
        self.obstacles_step(**kwargs)
        self.count += 1
        self.sampling = (self.count % (self.sample_time / self.step_time) == 0)

    def step_count(self, **kwargs):
        self.count += 1
        self.sampling = (self.count % (self.sample_time / self.step_time) == 0)

    def get_robot_list(self):
        return self.env_robot.robot_list

    def get_obstacle_list(self, obs_type=None):
        # obs_type： obstacle_circle； obstacle_polygon

        obs_list = []
        if obs_type is not None:
            [obs_list.extend(env_obs.obs_list) for env_obs in self.env_obstacle_list if env_obs.obs_class == obs_type]
        else:
            [obs_list.extend(env_obs.obs_list) for env_obs in self.env_obstacle_list]
        
        return obs_list

    def collision_check(self):
        collision_list = self.env_robot.collision_check_list(self.env_obstacle_list)
        return any(collision_list)

    def done(self, mode='all', collision_check=True) -> bool:
        # mode: any; any robot done, return done
        #       all; all robots done, return done
        done_list = self.done_list(collision_check)

        if mode == 'all':
            return all(done_list)
        elif mode == 'any':
            return any(done_list)

    def done_list(self, collision_check=True):

        arrive_flags = self.env_robot.arrive_list()

        if collision_check:
            self.collision_check()
            collision_flags = self.env_robot.collision_list()
        else:
            collision_flags = [False] * len(arrive_flags)
        
        done_list = [a or c for a, c in zip(arrive_flags, collision_flags)]
        return done_list
    
    def reset(self, done_list=None, mode='any'):
        # mode:  if done list is not None
        #   any: reset the env when any robot done
        #   all: reset env when all robots done
        #   single: reset one robot who has done, depending on the done list
        if done_list is None: self.reset_all()
        else:
            if mode == 'any' and any(done_list):
                self.reset_all()   
            elif mode == 'all' and all(done_list):
                self.reset_all()
            elif mode == 'single':
                [self.reset_single(i) for i, done in enumerate(done_list) if done]
    
    def reset_all(self):
        self.env_robot.reset()
        [env_obs.reset() for env_obs in self.env_obstacle_list if env_obs.dynamic]
    
    def reset_single(self, id):
        self.env_robot.reset(id)

    def render(self, pause_time=0.0001, **kwargs):
        
        if self.plot: 
            if self.sampling:
                self.draw_components(self.ax, mode='dynamic', **kwargs)
                plt.pause(pause_time)

                if self.save_ani: self.save_gif_figure(**kwargs)

                self.clear_components(self.ax, mode='dynamic', **kwargs)

    def render_once(self, pause_time=0.0001, **kwargs):
        if self.plot:
            self.draw_components(self.ax, mode='dynamic', **kwargs)
            plt.pause(pause_time)
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
            [env_obs.plot(ax, **kwargs) for env_obs in self.env_obstacle_list if not env_obs.dynamic]
        elif mode == 'dynamic':
            self.env_robot.plot(ax, **kwargs)
            [env_obs.plot(ax, **kwargs) for env_obs in self.env_obstacle_list if env_obs.dynamic]

        elif mode == 'all':
            self.env_robot.plot(ax, **kwargs)
            [env_obs.plot(ax, **kwargs) for env_obs in self.env_obstacle_list]
            # obstacle
        else:
            logging.error('error input of the draw mode')

    def clear_components(self, ax, mode='all', **kwargs):
        if mode == 'dynamic':
            self.env_robot.plot_clear(ax)
            [env_obs.plot_clear() for env_obs in self.env_obstacle_list if env_obs.dynamic]
            [line.pop(0).remove() for line in self.dyna_line_list]

            self.dyna_line_list = []
            
        elif mode == 'static':
            pass
        
        elif mode == 'all':
            plt.cla()

    def draw_trajectory(self, traj, style='g-', label='trajectory', show_direction=False, refresh=False, **kwargs):
        # traj: a list of points
        if isinstance(traj, list):
            path_x_list = [p[0, 0] for p in traj]
            path_y_list = [p[1, 0] for p in traj]
        elif isinstance(traj, np.ndarray):
            # raw*column: points * num
            path_x_list = [p[0] for p in traj.T]
            path_y_list = [p[1] for p in traj.T]

        line = self.ax.plot(path_x_list, path_y_list, style, label=label, **kwargs)

        if show_direction:
            if isinstance(traj, list):
                u_list = [cos(p[2, 0]) for p in traj]
                y_list = [sin(p[2, 0]) for p in traj]
            elif isinstance(traj, np.ndarray):
                u_list = [cos(p[2]) for p in traj.T]
                y_list = [sin(p[2]) for p in traj.T]

            self.ax.quiver(path_x_list, path_y_list, u_list, y_list)

        if refresh:
            self.dyna_line_list.append(line)

    def show(self, save_fig=False, fig_name='fig.png', **kwargs):
        if self.plot:
            self.draw_components(self.ax, mode='dynamic', **kwargs)
            
            if save_fig: self.fig.savefig(fig_name)

            logging.info('Program Done')
            plt.show()

    def end(self, ani_name='animation', save_fig=False, fig_name='fig.png', show=True, **kwargs):
        
        if self.save_ani: self.save_animate(ani_name)
            
        if self.plot:
            self.draw_components(self.ax, mode='dynamic', **kwargs)

            if save_fig: self.fig.savefig(fig_name)

            if show: plt.show()

    def save_gif_figure(self, save_figure_format='png', **kwargs):

        if not self.image_path.exists(): self.image_path.mkdir()

        order = str(self.count).zfill(3)
        plt.savefig(str(self.image_path)+'/'+order+'.'+save_figure_format, format=save_figure_format, **kwargs)

    def save_animate(self, ani_name='animated', keep_len=30, rm_fig_path=True):
        
        if not self.ani_path.exists(): self.ani_path.mkdir()
            
        images = list(self.image_path.glob('*.png'))
        images.sort()
        image_list = []
        for i, file_name in enumerate(images):

            if i == 0: continue

            image_list.append(imageio.imread(file_name))
            if i == len(images) - 1:
                for j in range(keep_len):
                    image_list.append(imageio.imread(file_name))

        imageio.mimsave(str(self.ani_path)+'/'+ ani_name+'.gif', image_list)
        print('Create animation successfully')

        if rm_fig_path: shutil.rmtree(self.image_path)


    # def reset(self, done_list=None, mode='all'):
    #     if done_list is None:
    #         self.env_robot.reset()
    #         [env_obs.reset() for env_obs in self.env_obstacle_list if env_obs.dynamic]

            



    


            