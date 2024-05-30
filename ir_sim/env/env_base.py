import yaml
from ir_sim.env.env_para import EnvPara
from ir_sim.util.util import file_check
from ir_sim.world import world
from .env_plot import EnvPlot
import threading
from ir_sim.global_param import world_param, env_param
import time
import sys
from ir_sim.world.object_factory import ObjectFactory
from matplotlib import pyplot as plt
import platform
import numpy as np
from pynput import keyboard
from .env_logger import EnvLogger
import copy


class EnvBase:

    '''
    The base class of environment.

        parameters:
            world_name: the name of the world file, default is None
     
    '''
    def __init__(self, world_name=None, display=True, disable_all_plot=False, save_ani=False, full=False, log=True, log_file='ir_sim.log', log_level='INFO'):

        env_para = EnvPara(world_name)
        object_factory = ObjectFactory() 
    
        # init env setting
        self.display = display
        self.disable_all_plot = disable_all_plot
        self.save_ani = save_ani
        self.logger = EnvLogger(log_file, log_level) 
        env_param.logger = self.logger 

        # init objects (world, obstacle, robot)
        self._world = world(world_name, **env_para.parse['world'])

        self._robot_collection = object_factory.create_from_parse(env_para.parse['robot'], 'robot')
        self._obstacle_collection = object_factory.create_from_parse(env_para.parse['obstacle'], 'obstacle')
        self._map_collection = object_factory.create_from_map(self._world.obstacle_positions, self._world.buffer_reso)
        self._object_collection = self._robot_collection + self._obstacle_collection + self._map_collection

        # env parameters
        self._env_plot = EnvPlot(self._world.grid_map, self.objects, self._world.x_range, self._world.y_range, **env_para.parse['plot'])
        env_param.objects = self.objects
        
        if world_param.control_mode == 'keyboard':
            self.init_keyboard(env_para.parse['keyboard'])

        if full:
            mode = platform.system()
            if mode == 'Linux':
                # mng = plt.get_current_fig_manager()
                plt.get_current_fig_manager().full_screen_toggle()
                # mng.resize(*mng.window.maxsize())
                # mng.frame.Maximize(True)

            elif mode == 'Windows':
                # figManager = plt.get_current_fig_manager()
                # figManager.window.showMaximized()
                # figManager.resize(*figManager.window.maxsize())
                # self._env_plot.fig.canvas.manager.window.showMaximized()
                mng = plt.get_current_fig_manager()
                mng.full_screen_toggle()
        

    ## magic methods
    def __del__(self):
        # self.logger.info('Simulated Environment End')
        print('Simulated Environment End with sim time elapsed: {} seconds'.format(round(self._world.time, 2)))

    def __str__(self):
        return f'Environment: {self._world.name}'

    ## step
    def step(self, action=None, action_id=0, **kwargs):

        if isinstance(action, list):
            self.objects_step(action)
        else:
            if world_param.control_mode == 'keyboard': 
                self.object_step(self.key_vel, self.key_id)
            else:
                self.object_step(action, action_id)

        # if action is None:
        #     action = [None] * len(self.objects)
        self._world.step()

    def objects_step(self, action=None):
        action = action + [None] * (len(self.objects) - len(action))
        [ obj.step(action) for obj, action in zip(self.objects, action)]

    def object_step(self, action, obj_id=0):
        self.objects[obj_id].step(action)
        [ obj.step() for obj in self.objects if obj._id != obj_id]


    ## render     
    def render(self, interval=0.05, figure_kwargs=dict(), **kwargs):

        # figure_args: arguments when saving the figures for animation, see https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.savefig.html for detail
        # default figure arguments

        if not self.disable_all_plot: 
            if self._world.sampling:

                if self.display: plt.pause(interval)

                if self.save_ani: self._env_plot.save_gif_figure(**figure_kwargs)

                self._env_plot.clear_components('dynamic', self.objects, **kwargs)
                self._env_plot.draw_components('dynamic', self.objects, **kwargs)
    

    def render_offline(self, interval=0.05, record=[], figure_kwargs=dict(), **kwargs):

        # figure_args: arguments when saving the figures for animation, see https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.savefig.html for detail
        # default figure arguments

        for objs in record:

            if not self.disable_all_plot: 
                if self._world.sampling:

                    if self.display: plt.pause(interval)

                    if self.save_ani: self._env_plot.save_gif_figure(**figure_kwargs)

                    self._env_plot.clear_components('dynamic', objs, **kwargs)
                    self._env_plot.draw_components('dynamic', objs, **kwargs)
                

    def show(self):
        self._env_plot.show()

    
    def reset_plot(self):
        plt.cla()
        self._env_plot.init_plot(self._world.grid_map, self.objects)


    # draw
    def draw_trajectory(self, traj, traj_type='g-', **kwargs):
        self._env_plot.draw_trajectory(traj, traj_type, **kwargs)

    def draw_points(self, points, s=30, c='b', **kwargs):
        self._env_plot.draw_points(points, s, c, **kwargs)


    def draw_box(self, vertex, refresh=True, **kwargs):
        self._env_plot.draw_box(vertex, refresh, **kwargs)


    ## keyboard control
    def init_keyboard(self, keyboard_kwargs=dict()):

        vel_max = keyboard_kwargs.get('vel_max', [3.0, 1.0])
        self.key_lv_max = keyboard_kwargs.get("key_lv_max", vel_max[0])
        self.key_ang_max = keyboard_kwargs.get("key_ang_max", vel_max[1])
        self.key_lv = keyboard_kwargs.get("key_lv", 0.0)
        self.key_ang = keyboard_kwargs.get("key_ang", 0.0)
        self.key_id = keyboard_kwargs.get("key_id", 0)
        self.alt_flag = 0

        plt.rcParams['keymap.save'].remove('s')
        plt.rcParams['keymap.quit'].remove('q')
        
        self.key_vel = np.zeros((2, 1))

        print('start to keyboard control')
        print('w: forward', 's: back forward', 'a: turn left', 'd: turn right', 
                'q: decrease linear velocity', 'e: increase linear velocity',
                'z: decrease angular velocity', 'c: increase angular velocity',
                'alt+num: change current control robot id', 'r: reset the environment')
                
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()


    
    def end(self, ending_time=3, **kwargs):

        if self.save_ani:
            self._env_plot.save_animate(**kwargs)

        self.logger.info(f'Figure will be closed within {ending_time:d} seconds.')
        plt.pause(ending_time)
        plt.close()
        

    def done(self, mode='all'):

        done_list = [ obj.done() for obj in self.objects if obj.role=='robot']

        if len(done_list) == 0:
            return False

        if mode == 'all':
            return all(done_list)
        elif mode == 'any':
            return any(done_list)
        
    def reset(self):
        self.reset_all() 

    def reset_all(self):
        [obj.reset() for obj in self.objects]
        

    def get_robot_info(self, id=0):
        return self.robot_list[id].get_info()

   
    @property
    def arrive(self, id=None, mode=None):

        if id is not None:
            assert isinstance(id, int), 'id should be integer'
            return self.robot_list[id].arrive
        else:
            assert mode in ['all', 'any'], 'mode should be all or any'
            return all([obj.arrive for obj in self.robot_list]) if mode == 'all' else any([obj.arrive for obj in self.robot_list])
    
    @property
    def collision(self, id=None, mode=None):
        
        if id is not None:
            assert isinstance(id, int), 'id should be integer'
            return self.robot_list[id].collision
        else:
            assert mode in ['all', 'any'], 'mode should be all or any'
            return all([obj.collision for obj in self.robot_list]) if mode == 'all' else any([obj.collision for obj in self.robot_list])


    @property
    def robot_list(self):
        return [ obj for obj in self.objects if obj.role == 'robot']

    @property
    def robot(self):
        return self.robot_list[0]
    
    def get_robot_state(self):
        return self.robot._state
    
    def get_lidar_scan(self, id=0):
        return self.robot_list[id].get_lidar_scan()
    
    @property
    def objects(self):
        return self._object_collection

    # region: keyboard control
    def on_press(self, key):

        try:
            if key.char.isdigit() and self.alt_flag:

                if int(key.char) >= self.robot_number:
                    print('out of number of robots')
                    self.key_id = int(key.char)
                else:
                    print('current control id: ', int(key.char))
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
            
            try:
                if "alt" in key.name:
                    self.alt_flag = True

            except AttributeError:

                if key.char.isdigit() and self.alt_flag:

                    if int(key.char) >= self.robot_number:
                        print('out of number of robots')
                        self.key_id = int(key.char)
                    else:
                        print('current control id: ', int(key.char))
                        self.key_id = int(key.char)
        
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
    

    


