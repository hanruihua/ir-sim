import logging
import os
import sys
import yaml
import imageio
import shutil
import platform
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

from pathlib import Path, PurePath
from pynput import keyboard
from math import sin, cos, pi
from .env_robot import EnvRobot
from ir_sim2.env import env_global
from .env_obstacle import EnvObstacle
from ir_sim2.world import world, RobotDiff, RobotAcker, RobotOmni, ObstacleCircle, ObstaclePolygon, ObstacleBlock, ObstacleLine

class EnvBase:

    robot_factory={'robot_diff': RobotDiff, 'robot_acker': RobotAcker, 'robot_omni': RobotOmni}
    obstacle_factory = {'obstacle_circle': ObstacleCircle, 'obstacle_block': ObstacleBlock, 'obstacle_polygon': ObstaclePolygon, 'obstacle_line': ObstacleLine}

    def __init__(self, world_name=None, control_mode='auto', collision_mode='stop', 
        disable_all_plot=False, display=True, save_fig=False, save_ani=False, full=False, 
        custom_robot=None, subplot_num=1, **kwargs) -> None:
        
        '''
        The main environment class for this simulator

        world_name: path of the yaml
        control_mode: 
            auto: receive the velocity command to move 
            keyboard: receive the velocity from the keyboard to move 
        
        collision_mode: 
            None: No collision check
            stop (default): All Objects stop when collision, 
            react: robot will have reaction when collision with others  (only work for the circular robot in current version)

        disable_all_plot: True or False (default), if True, all parts related to plot (figure or animation) are disabled.
        display: True or False to display the figure
        save_ani: True or False to save the animation
        full: Tru or false to whether show the figure full screen
        custom_robot: custom robot class for user;
        subplot_num: only 0, 1, 2, 3; default: 1
        '''

        # arguments
            # world_args: arguments of the world, including width, length...
            # robot_args: arguments of the multiple robots, including number, type...
            # keyboard_args: arguments of the keyboard control, including key_lv_max....

        world_args, robot_args, keyboard_args, init_args = dict(), dict(), dict(), dict()
        obstacle_args_list = []
        
        # path and file configuration
        world_file_path = EnvBase.file_check(world_name)


        if world_file_path != None:
           
            with open(world_file_path) as file:
                com_list = yaml.load(file, Loader=yaml.FullLoader)
                world_args = com_list.get('world', dict())
                robot_args = com_list.get('robots', dict())
                keyboard_args = com_list.get('keyboard', dict())
                obstacle_args_list = com_list.get('obstacles', [])

        world_args.update(kwargs.get('world_args', dict()))
        robot_args.update(kwargs.get('robot_args', dict()))
        keyboard_args.update(kwargs.get('keyboard_args', dict()))
        [obstacle_args.update(kwargs.get('obstacle_args', dict())) for obstacle_args in obstacle_args_list]
        init_args.update(kwargs.get('init_args', dict()))

        # world arguments
        self.world = world(**world_args)
        self.step_time = self.world.step_time

        # robot arguments
        self.robot_args = robot_args
        self.custom_robot = custom_robot

        # obstacle arguments
        self.obstacle_args_list = obstacle_args_list

        # plot
        self.disable_all_plot = disable_all_plot
        self.display = display
        self.dyna_line_list = []
        self.dyna_patch_list = []
        self.subplot_num = subplot_num

        # Mode
        self.control_mode = control_mode 
        self.collision_mode = collision_mode 
        
        # animation arguments:
        self.save_ani = save_ani  
        self.ani_dpi = kwargs.get('ani_dpi', 300)
        self.save_fig = save_fig 
        self.fig_dpi = kwargs.get('fig_dpi', 600)
        self.bbox_inches = kwargs.get('bbox_inches', 'tight') 

        # configure path
        root_path = kwargs.get('root_path', Path(sys.path[0]))
        ani_buffer_path = kwargs.get('ani_buffer_path', Path(sys.path[0] + '/' + 'animation_buffer'))
        ani_path = kwargs.get('ani_path', Path(sys.path[0] + '/' + 'animation'))
        fig_path = kwargs.get('fig_path', Path(sys.path[0] + '/' + 'fig'))

        self.root_path = EnvBase.path_to_pure(root_path)
        self.ani_buffer_path = EnvBase.path_to_pure(ani_buffer_path)
        self.ani_path = EnvBase.path_to_pure(ani_path)
        self.fig_path = EnvBase.path_to_pure(fig_path)
        
        # initialize the environment
        self._init_environment(**init_args)

        if control_mode == 'keyboard':
            vel_max = self.robot_args.get('vel_max', [2.0, 2.0])
            self.key_lv_max = keyboard_args.get("key_lv_max", vel_max[0])
            self.key_ang_max = keyboard_args.get("key_ang_max", vel_max[1])
            self.key_lv = keyboard_args.get("key_lv", 0.0)
            self.key_ang = keyboard_args.get("key_ang", 0.0)
            self.key_id = keyboard_args.get("key_id", 1)
            self.alt_flag = 0

            plt.rcParams['keymap.save'].remove('s')
            plt.rcParams['keymap.quit'].remove('q')
            
            self.key_vel = np.zeros((2, 1))

            print('start to keyboard control')
            print('w: forward', 's: backforward', 'a: turn left', 'd: turn right', 
                  'q: decrease linear velocity', 'e: increase linear velocity',
                  'z: decrease angular velocity', 'c: increase angular velocity',
                  'alt+num: change current control robot id', 'r: reset the environment')
                  
            self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
            self.listener.start()

        # full screen
        if full:
            mode = platform.system()
            if mode == 'Linux':
                # mng = plt.get_current_fig_manager()
                plt.get_current_fig_manager().full_screen_toggle()
                # mng.resize(*mng.window.maxsize())
                # mng.frame.Maximize(True)

            elif mode == 'Windows':
                figManager = plt.get_current_fig_manager()
                figManager.window.showMaximized()

    # region: initialization
    def _init_environment(self, **kwargs): 
        # kwargs: 
        #        
        # 
        
        # default obstacles
        self.env_obstacle_list = [EnvObstacle(self.obstacle_factory[oa['type']], step_time=self.step_time, **oa) for oa in self.obstacle_args_list]
        self.obstacle_list = [obs for eol in self.env_obstacle_list for obs in eol.obs_list]
        
        env_global.obstacle_list = self.obstacle_list

        # default robots
        robot_type = self.robot_args.get('type', 'robot_diff')  

        if robot_type == 'robot_custom':
            self.env_robot = EnvRobot(self.custom_robot, step_time=self.step_time, **self.robot_args)
        else:
            self.env_robot = EnvRobot(self.robot_factory[robot_type], step_time=self.step_time, **self.robot_args)
        
        # default robots 
        self.robot_list = self.env_robot.robot_list
        self.robot = self.robot_list[0] if len(self.robot_list) > 0 else None
        self.robot_number = len(self.robot_list)
       
        self.components = self.robot_list + self.obstacle_list

        # global objects through multiple files
        env_global.robot_list = self.robot_list
        env_global.components = self.components
        env_global.control_mode = self.control_mode
        env_global.collision_mode = self.collision_mode

        # plot
        if not self.disable_all_plot and self.subplot_num!=0:
            # self.fig, self.ax = plt.subplots()
            # self.init_plot(self.ax, **kwargs)

            self.fig = plt.figure()

            if self.subplot_num == 1:
                self.ax = self.fig.add_subplot(111) # main axis

            elif self.subplot_num == 2:
                self.ax = self.fig.add_subplot(121) # main axis
                self.ax2 = self.fig.add_subplot(122)

                self.init_sub_plot(self.ax2)

            elif self.subplot_num == 3:

                gs = GridSpec(2, 3)

                self.ax = self.fig.add_subplot(gs[:, 0:2]) # main axis
                self.ax2 = self.fig.add_subplot(gs[0, -1])
                self.ax3 = self.fig.add_subplot(gs[1, -1])

                self.init_sub_plot(self.ax2)
                self.init_sub_plot(self.ax3)
            
            # self.fig.tight_layout()
            self.fig.align_labels()
            self.init_plot(self.ax, **kwargs)
            
    # endregion: initialization  

    # region: step forward
    def step(self, vel_list=[], **kwargs):

        if self.control_mode == 'keyboard':
            self.robots_step(self.key_vel, vel_id=self.key_id, **kwargs)
        else:
            self.robots_step(vel_list, **kwargs)

        self.obstacles_step(**kwargs)
        self.world.step()

        env_global.time_increment()

    def robots_step(self, vel_list, **kwargs):
        self.env_robot.move(vel_list, **kwargs)
    
    def obstacles_step(self, **kwargs):
        [ env_obs.move() for env_obs in self.env_obstacle_list if env_obs.dynamic]

    def cal_des_vel(self, **kwargs):
        return self.env_robot.cal_des_vel(**kwargs)

    # endregion: step forward  

    # region: get information
    def get_world_size(self):
        return self.world.height, self.world.width

    # get information
    def get_robot_list(self):
        return self.env_robot.robot_list

    def get_robot_info(self, id=0):
        return self.env_robot.robot_list[id].get_info()
    
    def get_robot_estimation(self, id=0):
        # when noise True
        return self.env_robot.robot_list[id].odometry

    def get_obstacle_list(self, obs_type=None):
        # obs_type： obstacle_circle； obstacle_polygon

        obs_list = []
        if obs_type is not None:
            [obs_list.extend(env_obs.obs_list) for env_obs in self.env_obstacle_list if env_obs.obs_class == obs_type]
        else:
            [obs_list.extend(env_obs.obs_list) for env_obs in self.env_obstacle_list]
        
        return obs_list

    def get_lidar_scan(self, id=0):
        return self.env_robot.robot_list[id].get_lidar_scan()
    
    def get_landmarks(self, id=0):
        return self.env_robot.robot_list[id].get_landmarks()

    def get_ax(self, ax_id=0):
        return ax_id

    # endregion: get information

    # region: check status
    # def collision_check(self):
    #     collision_list = self.env_robot.collision_check_list(self.env_obstacle_list)
    #     return any(collision_list)
    
    def collision_status(self):
        return self.env_robot.collision_status()

    def collision_status_list(self):
        return self.env_robot.collision_list()

    # 
    def done(self, mode='all', collision_check=True) -> bool:
        # mode: any; any robot done, return done
        #       all; all robots done, return done
        done_list = self.done_list(collision_check)

        if mode == 'all':
            return all(done_list)
        elif mode == 'any':
            return any(done_list)

    def done_list(self, collision_check=True, **kwargs):

        arrive_flags = self.env_robot.arrive_list()

        if collision_check:
            collision_flags = self.env_robot.collision_list()
        else:
            collision_flags = [False] * len(arrive_flags)
        
        done_list = [a or c for a, c in zip(arrive_flags, collision_flags)]
        return done_list
    # endregion: check status

    # region: reset the environment
    def reset(self, mode='now', **kwargs):
        # mode: 
        #   default: reset the env now
        #   any: reset all the env when any robot done
        #   all: reset all the env when all robots done
        #   single: reset one robot who has done, depending on the done list
        if mode == 'now':
            self.reset_all() 
        else:
            done_list = self.done_list(**kwargs)
            if mode == 'any' and any(done_list): self.reset_all()
            elif mode == 'all' and all(done_list): self.reset_all()
            elif mode == 'single': 
                [self.reset_single(i) for i, done in enumerate(done_list) if done]
    
    def reset_all(self):
        self.env_robot.reset()
        [env_obs.reset() for env_obs in self.env_obstacle_list if env_obs.dynamic]
    
    def reset_single(self, id):
        self.env_robot.reset(id)
    # endregion: reset the environment

    # region: environment render
    def render(self, pause_time=0.05, fig_kwargs=dict(), **kwargs):
        # figure_args: arguments when saving the figures for animation, see https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.savefig.html for detail
        # default figure arguments
        
        if not self.disable_all_plot: 
            if self.world.sampling:
                self.draw_components(self.ax, mode='dynamic', **kwargs)
                
                if self.display: plt.pause(pause_time)

                if self.save_ani: self.save_gif_figure(bbox_inches=self.bbox_inches, dpi=self.ani_dpi, **fig_kwargs)

                self.clear_components(self.ax, mode='dynamic', **kwargs)

    def render_once(self, pause_time=0.0001, **kwargs):
        if not self.disable_all_plot:
            self.draw_components(self.ax, mode='dynamic', **kwargs)
            plt.pause(pause_time)
            self.clear_components(self.ax, mode='dynamic', **kwargs)
      
    def init_plot(self, ax, **kwargs):
        
        ax.set_aspect('equal') 
        ax.set_xlim(self.world.x_range) 
        ax.set_ylim(self.world.y_range)
        
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")

        self.draw_components(ax, mode='static', **kwargs)
    
    def init_sub_plot(self, ax, **kwargs):
        
        ax.set_aspect('equal') 
        ax.set_xlim(self.world.x_range) 
        ax.set_ylim(self.world.y_range)
        

        
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

    def draw_trajectory(self, traj, traj_type='g-', label='trajectory', show_direction=False, refresh=False, **kwargs):
        # traj: a list of points
        if isinstance(traj, list):
            path_x_list = [p[0, 0] for p in traj]
            path_y_list = [p[1, 0] for p in traj]
        elif isinstance(traj, np.ndarray):
            # raw*column: points * num
            path_x_list = [p[0] for p in traj.T]
            path_y_list = [p[1] for p in traj.T]

        if not self.disable_all_plot: 
            line = self.ax.plot(path_x_list, path_y_list, traj_type, label=label, **kwargs)

        if show_direction:
            if isinstance(traj, list):
                u_list = [cos(p[2, 0]) for p in traj]
                y_list = [sin(p[2, 0]) for p in traj]
            elif isinstance(traj, np.ndarray):
                u_list = [cos(p[2]) for p in traj.T]
                y_list = [sin(p[2]) for p in traj.T]

            self.ax.quiver(path_x_list, path_y_list, u_list, y_list)

        if refresh and not self.disable_all_plot: 
            self.dyna_line_list.append(line)

    # def draw_data(self, data, new_figure=True, show=True, label='default', **kwargs):
    #     # data: list of values
    #     if new_figure:
    #         fig, ax = plt.subplots()
    #     else:
    #         ax = self.ax

    #     ax.plot(data, label=label, **kwargs)
    #     if show: plt.show()

    def draw_uncertainty(self, mean, std, scale=20, facecolor='gray', refresh=True, **kwargs):
        
        x = mean[0, 0]
        y = mean[1, 0]

        std_x = std[0, 0]
        std_y = std[1, 0]
        
        theta = mean[2, 0] if mean.shape[0] > 2 else 0

        angle = theta * (180 / pi)
        
        ellipse = mpl.patches.Ellipse(xy=(x, y), width=scale*std_x, height=scale*std_y, angle=angle, facecolor=facecolor, **kwargs)
        ellipse.set_zorder(1)
        self.ax.add_patch(ellipse)
            
        if refresh:
            self.dyna_patch_list.append(ellipse)

    def clear_components(self, ax, mode='all', **kwargs):
        if mode == 'dynamic':
            self.env_robot.plot_clear(ax)
            [env_obs.plot_clear() for env_obs in self.env_obstacle_list if env_obs.dynamic]
            [line.pop(0).remove() for line in self.dyna_line_list]
            [patch.remove() for patch in self.dyna_patch_list]

            self.dyna_line_list = []
            self.dyna_patch_list = []

        if not kwargs.get('show_text', True):
            self.ax.texts.clear()

        elif mode == 'static':
            pass
        
        elif mode == 'all':
            plt.cla()
    # endregion: environment render

    # region: animation
    def save_gif_figure(self, save_figure_format='png', **kwargs):

        if not self.ani_buffer_path.exists(): self.ani_buffer_path.mkdir()

        order = str(self.world.count).zfill(3)
        self.fig.savefig(str(self.ani_buffer_path)+'/'+order+'.'+save_figure_format, format=save_figure_format, **kwargs)

    def save_animate(self, ani_name='animated', suffix='.gif', keep_len=30, rm_fig_path=True, **kwargs):
        
        print('Start to create animation')

        if not self.ani_path.exists(): self.ani_path.mkdir()
            
        images = list(self.ani_buffer_path.glob('*.png'))
        images.sort()
        image_list = []
        for i, file_name in enumerate(images):

            if i == 0: continue

            image_list.append(imageio.imread(str(file_name)))
            if i == len(images) - 1:
                for j in range(keep_len):
                    image_list.append(imageio.imread(str(file_name)))

        imageio.mimsave(str(self.ani_path)+'/'+ ani_name + suffix, image_list, **kwargs)
        print('Create animation successfully')

        if rm_fig_path: shutil.rmtree(self.ani_buffer_path)
    # endregion: animation

    # region: keyboard control
    def on_press(self, key):

        try:
            if key.char.isdigit() and self.alt_flag:

                if int(key.char) > self.robot_number:
                    print('out of number of robots')
                else:
                    self.key_id = int(key.char)

            if key.char == 'w':
                self.key_lv = self.key_lv_max
            if key.char == 's':
                self.key_lv = - self.key_lv_max
            if key.char == 'a':
                self.key_ang = self.key_ang_max
            if key.char == 'd':
                self.key_ang = -self.key_ang_max
            
            self.key_vel = np.array([ [self.key_lv], [self.key_ang]])

        except AttributeError:
            if "alt" in key.name:
                self.alt_flag = True
                
    def on_release(self, key):
        
        try:
            if key.char == 'w':
                self.key_lv = 0
            if key.char == 's':
                self.key_lv = 0
            if key.char == 'a':
                self.key_ang = 0
            if key.char == 'd':
                self.key_ang = 0
            if key.char == 'q':
                self.key_lv_max = self.key_lv_max - 0.2
                print('current lv ', self.key_lv_max)
            if key.char == 'e':
                self.key_lv_max = self.key_lv_max + 0.2
                print('current lv ', self.key_lv_max)
            
            if key.char == 'z':
                self.key_ang_max = self.key_ang_max - 0.2
                print('current ang ', self.key_ang_max)
            if key.char == 'c':
                self.key_ang_max = self.key_ang_max + 0.2
                print('current ang ', self.key_ang_max)
            
            if key.char == 'r':
                self.reset()
            
            self.key_vel = np.array([[self.key_lv], [self.key_ang]])

        except AttributeError:
            if "alt" in key.name:
                self.alt_flag = False
    # endregion:keyboard control

    # region: the end of the environment loop 
    def end(self, ani_name='animation', fig_name='fig.png', ending_time = 3, suffix='.gif', keep_len=30, rm_fig_path=True, fig_kwargs=dict(), ani_kwargs=dict(), **kwargs):
        
        # fig_kwargs: arguments when saving the figures for animation, see https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.savefig.html for detail
        # ani_kwargs: arguments for animations(gif): see https://imageio.readthedocs.io/en/v2.8.0/format_gif-pil.html#gif-pil for detail
        print('DONE')

        show = kwargs.get('show', self.display)
        
        if not self.disable_all_plot:

            if self.save_ani:
                self.save_animate(ani_name, suffix, keep_len, rm_fig_path, **ani_kwargs)

            if self.save_fig or show:
                self.draw_components(self.ax, mode='dynamic', **kwargs)

            if self.save_fig: 
                self.fig.savefig(str(self.fig_path) + fig_name, bbox_inches=self.bbox_inches, dpi=self.fig_dpi, **fig_kwargs)

            if show: 
                plt.show(block=False)
                print(f'Figure will be closed within {ending_time:d} seconds.')
                plt.pause(ending_time)
                plt.close()


    # endregion: the end of the environment loop  

    @staticmethod
    def path_to_pure(path):
        # check whether path exist or the type is correct
        # pathtype: str or pure

        if isinstance(path, PurePath):
            real_path = path
        elif isinstance(path, str):
            real_path = Path(path)
        else:
            print('error: wrong path type')
        
        return real_path

    @staticmethod
    def file_check(file_name):
        # check whether file exist or the type is correct

        if os.path.exists(file_name):
            abs_file_name = file_name
        elif os.path.exists(sys.path[0] + '/' + file_name):
            abs_file_name = sys.path[0] + '/' + file_name
        elif os.path.exists(os.getcwd() + '/' + file_name):
            abs_file_name = os.getcwd() + '/' + file_name
        else:
            print('WARNING: No World File Found, Using default arguments')
            abs_file_name = None

        return abs_file_name

        

    


            