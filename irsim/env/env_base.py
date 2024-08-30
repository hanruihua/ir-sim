import yaml
from irsim.env.env_config import EnvConfig
from irsim.util.util import file_check
from irsim.world import world
from .env_plot import EnvPlot
from irsim.global_param import world_param, env_param
from irsim.world.object_factory import ObjectFactory
from matplotlib import pyplot as plt
import platform
import numpy as np
from pynput import keyboard
from .env_logger import EnvLogger
from irsim.lib.generation import random_generate_polygon
from shapely import Polygon

class EnvBase:

    '''
    The base class of environment. This class will read the yaml file and create the world, robot, obstacle, and map objects.   

    Args:
        world_name (str): Path to the world yaml file.
        display (bool): Flag to display the environment.
        disable_all_plot (bool): Flag to disable all plots and figures.
        save_ani (bool): Flag to save the animation.
        full (bool): Flag to full screen the figure.
        log_file (str): Name of the log file.
        log_level (str): Level of the log output.
    '''

    def __init__(self, world_name: str=None, display: bool=True, disable_all_plot: bool=False, save_ani: bool=False, full: bool=False, log_file: str=None, log_level: str='INFO') -> None:

        env_config = EnvConfig(world_name)
        object_factory = ObjectFactory() 
    
        # init env setting
        self.display = display
        self.disable_all_plot = disable_all_plot
        self.save_ani = save_ani
        self.logger = EnvLogger(log_file, log_level) 
        env_param.logger = self.logger 

        # init objects (world, obstacle, robot)
        self._world = world(world_name, **env_config.parse['world'])

        self._robot_collection = object_factory.create_from_parse(env_config.parse['robot'], 'robot')
        self._obstacle_collection = object_factory.create_from_parse(env_config.parse['obstacle'], 'obstacle')
        self._map_collection = object_factory.create_from_map(self._world.obstacle_positions, self._world.buffer_reso)
        self._object_collection = self._robot_collection + self._obstacle_collection + self._map_collection

        # env parameters
        self._env_plot = EnvPlot(self._world.grid_map, self.objects, self._world.x_range, self._world.y_range, **env_config.parse['plot'])
        env_param.objects = self.objects
        
        if world_param.control_mode == 'keyboard':
            self.init_keyboard(env_config.parse['keyboard'])

        if full:
            system_platform = platform.system()
            if system_platform == 'Linux':
                plt.get_current_fig_manager().full_screen_toggle()

            elif system_platform == 'Windows':
                mng = plt.get_current_fig_manager()
                mng.full_screen_toggle()
        

    def __del__(self):
        print('Simulated Environment End with sim time elapsed: {} seconds'.format(round(self._world.time, 2)))

    def __str__(self):
        return f'Environment: {self._world.name}'


    def step(self, action=None, action_id=0):

        '''
        Perform a simulation step in the environment.

        Args:
            if action is list:
                List of actions to be performed for each robot in the environment.
            if action is numpy array (2 * 1 vector): 
                differential robot action:  linear velocity, angular velocity
                omnidirectional robot action: velocity in x, velocity in y
                Ackermann robot action: linear velocity, Steering angle

            action_id (int 0): Apply the action to the robot with the given id.
        '''

        if isinstance(action, list):
            self._objects_step(action)
        else:
            if world_param.control_mode == 'keyboard': 
                self._object_step(self.key_vel, self.key_id)
            else:
                self._object_step(action, action_id)

        self._world.step()

    def _objects_step(self, action=None):
        action = action + [None] * (len(self.objects) - len(action))
        [ obj.step(action) for obj, action in zip(self.objects, action)]

    def _object_step(self, action, obj_id=0):
        self.objects[obj_id].step(action)
        [ obj.step() for obj in self.objects if obj._id != obj_id]

    ## render     
    def render(self, interval=0.05, figure_kwargs=dict(), **kwargs):

        '''
        Render the environment.

        Args:
            interval (float): Time interval between frames in seconds.
            figure_kwargs (dict): Additional keyword arguments for saving figures,  see https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.savefig.html for detail.
            kwargs: Additional keyword arguments for drawing components. see object_base.plot() function for detail.
        '''

        if not self.disable_all_plot: 
            if self._world.sampling:

                if self.display: plt.pause(interval)

                if self.save_ani: self._env_plot.save_gif_figure(**figure_kwargs)

                self._env_plot.clear_components('dynamic', self.objects)
                self._env_plot.draw_components('dynamic', self.objects, **kwargs)
    
    def show(self):

        '''
        Show the environment figure.
        '''

        self._env_plot.show()

    # draw various components
    def draw_trajectory(self, traj, traj_type='g-', **kwargs):

        '''
        Draw the trajectory on the environment figure.      

        Args:
            traj (list): List of trajectory points (2 * 1 vector).     
            traj_type: Type of the trajectory line, see matplotlib plot function for detail.     
            **kwargs: Additional keyword arguments for drawing the trajectory, see env_plot.draw_trajectory() function for detail.      
        '''

        self._env_plot.draw_trajectory(traj, traj_type, **kwargs)

    def draw_points(self, points, s=30, c='b', refresh=True, **kwargs):

        '''
        Draw points on the environment figure.

        Args:
            points (list): List of points (2*1) to be drawn.  
            s (int): Size of the points.   
            c (str): Color of the points.    
            refresh (bool): Flag to refresh the points in the figure.       
            **kwargs: Additional keyword arguments for drawing the points, see ax.scatter (https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.scatter.html) function for detail.      
        '''

        self._env_plot.draw_points(points, s, c, refresh, **kwargs)

    def draw_box(self, vertex, refresh=True, color='-b'):

        '''
        Draw a box by the vertices.

        Args:
            vertices: matrix of vertices, point_dim*vertex_num
            refresh: whether to refresh the plot, default False
            color: color of the box, default 'b-'
        ''' 
        self._env_plot.draw_box(vertex, refresh, color)

    ## keyboard control
    def init_keyboard(self, keyboard_kwargs=dict()):

        '''
        Initialize keyboard control for the environment.

        Args:
            keyboard_kwargs (dict): Dictionary of keyword arguments for keyboard control settings.
                - vel_max (list): Maximum velocities [linear, angular]. Default is [3.0, 1.0].
                - key_lv_max (float): Maximum linear velocity. Default is vel_max[0].
                - key_ang_max (float): Maximum angular velocity. Default is vel_max[1].
                - key_lv (float): Initial linear velocity. Default is 0.0.
                - key_ang (float): Initial angular velocity. Default is 0.0.
                - key_id (int): Initial robot control ID. Default is 0.
            
            Keys:
                w: Move forward.
                s: Move backward.
                a: Turn left.
                d: Turn right.
                q: Decrease linear velocity.
                e: Increase linear velocity.
                z: Decrease angular velocity.
                c: Increase angular velocity.
                alt + num: Change the current control robot id.
                r: Reset the environment.   
        '''

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
                
        self.listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        self.listener.start()


    
    def end(self, ending_time=3.0, **kwargs):

        '''
        End the simulation, save the animation, and close the environment.

        Args:
            ending_time (float): Time in seconds to wait before closing the figure, default is 3 seconds.
            **kwargs: Additional keyword arguments for saving the animation, see env_plot.save_animate() function for detail.
        '''

        if self.save_ani:
            self._env_plot.save_animate(**kwargs)

        self.logger.info(f'Figure will be closed within {ending_time:.2f} seconds.')
        plt.pause(ending_time)
        plt.close()
        

    def done(self, mode='all'):

        '''
        Check if the simulation is done.

        Args:
            mode (str): Mode to check if all or any of the objects are done.
                - all (str): Check if all objects are done.
                - any (str): Check if any of the objects are done.
        '''

        done_list = [ obj.done() for obj in self.objects if obj.role=='robot']

        if len(done_list) == 0:
            return False

        if mode == 'all':
            return all(done_list)
        elif mode == 'any':
            return any(done_list)
        
    def reset(self):

        '''
        Reset the environment.
        '''

        self._reset_all() 
        self.step(action=np.zeros((2, 1)))

    def _reset_all(self):
        [obj.reset() for obj in self.objects]


    def reset_plot(self):

        '''
        Reset the environment figure.
        '''

        plt.cla()
        self._env_plot.init_plot(self._world.grid_map, self.objects)

    # region: environment change
    def random_obstacle_position(self, range_low = [0, 0, -3.14], range_high = [10, 10, 3.14]):

        '''
        Random obstacle positions in the environment.

        Args:
            range_low (list [x, y, theta]): Lower bound of the random range for the obstacle states. Default is [0, 0, -3.14]. 
            range_high (list [x, y, theta]): Upper bound of the random range for the obstacle states. Default is [10, 10, 3.14].
        '''

        if isinstance(range_low, list):
            range_low = np.c_[range_low]
        
        if isinstance(range_high, list):
            range_high = np.c_[range_high]

        random_states = np.random.uniform(range_low, range_high, (3, self.obstacle_number))
            
        for i, obj in enumerate(self.obstacle_list):
            obj.set_state(random_states[:, i].reshape(3, 1), init=True)
        
        self._env_plot.clear_components('all', self.obstacle_list)
        self._env_plot.draw_components('all', self.obstacle_list)
    

    def random_polygon_shape(self, center_range = [0, 0, 10, 10], avg_radius_range = [0.1, 1], irregularity_range = [0, 1], spikeyness_range = [0, 1], num_vertices_range=[4, 10]):

        '''
        Random polygon shapes for the obstacles in the environment.

        Args:
            center_range (list): Range of the center of the polygon. Default is [0, 0, 10, 10].
            avg_radius_range (list): Range of the average radius of the polygon. Default is [0.1, 1].
            irregularity_range (list): Range of the irregularity of the polygon. Default is [0, 1].
            spikeyness_range (list): Range of the spikeyness of the polygon. Default is [0, 1].
            num_vertices_range (list): Range of the number of vertices of the polygon. Default is [4, 10].

            center (Tuple[float, float]):
                a pair representing the center of the circumference used
                to generate the polygon.
            avg_radius (float):
                the average radius (distance of each generated vertex to
                the center of the circumference) used to generate points
                with a normal distribution.
            irregularity (float): 0 - 1
                variance of the spacing of the angles between consecutive
                vertices.
            spikeyness (float): 0 - 1
                variance of the distance of each vertex to the center of
                the circumference.
            num_vertices (int):
                the number of vertices of the polygon.
        '''

        vertices_list = random_generate_polygon(self.obstacle_number, center_range, avg_radius_range, irregularity_range, spikeyness_range, num_vertices_range)

        for i, obj in enumerate(self.obstacle_list):

            if obj.shape == 'polygon':
                geom = Polygon(vertices_list[i])
                obj.set_init_geometry(geom)
        
            
        self._env_plot.clear_components('all', self.obstacle_list)
        self._env_plot.draw_components('all', self.obstacle_list)
            
    # endregion: environment change

        
    # region: get information
    def get_robot_state(self):

        '''
        Get the current state of the robot.

        Returns: 
            state: 3*1 vector [x, y, theta]
        
        '''

        return self.robot._state
    
    def get_lidar_scan(self, id=0):

        '''
        Get the lidar scan of the robot with the given id.

        Args:
            id (int): Id of the robot.
        
        Returns:
            Dict: Dict of lidar scan points, see lidar2d/get_scan() for detail.
        '''

        return self.robot_list[id].get_lidar_scan()
    
    def get_lidar_offset(self, id=0):

        '''
        Get the lidar offset of the robot with the given id.


        Args:
            id (int): Id of the robot.

        Returns:
            list of float: Lidar offset of the robot, [x, y, theta]
        '''

        return self.robot_list[id].get_lidar_offset()

    def get_obstacle_list(self):

        '''
        Get the information of the obstacles in the environment.

        Returns:
            list of dict: List of obstacle information, see Obstacle_Info in Object_base for detail.
        '''

        return [ obj.get_obstacle_info() for obj in self.objects if obj.role == 'obstacle']

    
    def get_robot_info(self, id=0):

        '''
        Get the information of the robot with the given id.

        Args:
            id (int): Id of the robot.
        
        Returns:
            see ObjectInfo in Object_base for detail
        '''

        return self.robot_list[id].get_info()

    # endregion: get information 

        
    #region: property
    @property
    def arrive(self, id=None, mode=None):
        '''
        Check if the robot(s) have arrived at their destination.

        Args:
            id (int, optional): Id of the specific robot to check. Default is None.
            mode (str, optional): Mode to check for all or any robots. Must be 'all' or 'any'. Default is None.

        Returns:
            bool: Arrival status of the specified robot or all/any robots.
        '''

        # If a specific robot id is provided
        if id is not None:
            # Ensure the id is an integer
            assert isinstance(id, int), 'id should be integer'
            # Return the arrival status of the specified robot
            return self.robot_list[id].arrive
        else:
            # Ensure the mode is either 'all' or 'any'
            assert mode in ['all', 'any'], 'mode should be all or any'
            # Return True if all robots have arrived, otherwise return True if any robot has arrived
            return all([obj.arrive for obj in self.robot_list]) if mode == 'all' else any([obj.arrive for obj in self.robot_list])

    @property
    def collision(self, id=None, mode=None):
        '''
        Check if the robot(s) have collided.

        Args:
            id (int, optional): Id of the specific robot to check. Default is None.
            mode (str, optional): Mode to check for all or any robots. Must be 'all' or 'any'. Default is None.

        Returns:
            bool: Collision status of the specified robot or all/any robots.
        '''

        # If a specific robot id is provided
        if id is not None:
            # Ensure the id is an integer
            assert isinstance(id, int), 'id should be integer'
            # Return the collision status of the specified robot
            return self.robot_list[id].collision
        else:
            # Ensure the mode is either 'all' or 'any'
            assert mode in ['all', 'any'], 'mode should be all or any'
            # Return True if all robots have collided, otherwise return True if any robot has collided
            return all([obj.collision for obj in self.robot_list]) if mode == 'all' else any([obj.collision for obj in self.robot_list])

    @property
    def robot_list(self):
        '''
        Get the list of robots in the environment.

        Returns:
            list: List of robot objects [].
        '''

        return [obj for obj in self.objects if obj.role == 'robot']
    
    @property
    def obstacle_list(self):
        '''
        Get the list of obstacles in the environment.

        Returns:
            list: List of obstacle objects.
        '''
        return [obj for obj in self.objects if obj.role == 'obstacle']
    


    @property
    def objects(self):
        return self._object_collection
    

    @property
    def step_time(self):
        return self._world.step_time


    @property
    def robot(self):
        return self.robot_list[0]


    @property
    def obstacle_number(self):
        return len(self.obstacle_list)
                

    #endregion: property


    # region: keyboard control
    def _on_press(self, key):

        '''
        Handle key press events for keyboard control.

        Args:
            key (pynput.keyboard.Key): The key that was pressed.
        '''

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
        
    def _on_release(self, key):
        
        '''
        Handle key release events for keyboard control.

        Args:
            key (pynput.keyboard.Key): The key that was released.
        '''
    
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
                print('current linear velocity', self.key_lv_max)
            if key.char == 'e':
                self.key_lv_max = self.key_lv_max + 0.2
                print('current linear velocity', self.key_lv_max)
            
            if key.char == 'z':
                self.key_ang_max = self.key_ang_max - 0.2
                print('current angular velocity ', self.key_ang_max)
            if key.char == 'c':
                self.key_ang_max = self.key_ang_max + 0.2
                print('current angular velocity ', self.key_ang_max)
            
            if key.char == 'r':
                self.reset()
            
            self.key_vel = np.array([[self.key_lv], [self.key_ang]])

        except AttributeError:
            if "alt" in key.name:
                self.alt_flag = False
    # endregion:keyboard control
    

    

