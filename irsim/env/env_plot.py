import matplotlib.pyplot as plt
import logging
from irsim.global_param.path_param import path_manager as pm
from irsim.global_param import world_param, env_param
import os
import imageio
import shutil
import glob
from math import sin, cos
import numpy as np


class EnvPlot:

    '''
    EnvPlot class for visualizing the environment.

    Attributes:
    -----------
    grid_map (optional): The grid map of the environment. Png file

    Objects: list of object in the environment.

    x_range : list
        The range of x-axis values. Default is [0, 10].
    y_range : list
        The range of y-axis values. Default is [0, 10].
    
    saved_figure : dict
        Keyword arguments for saving the figure.
        See https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.savefig.html for details.
    saved_ani : dict
        Keyword arguments for saving the animation.
        See https://imageio.readthedocs.io/en/v2.8.0/format_gif-pil.html#gif-pil for details.

    kwargs:
        color_map : dict
            Color map for different objects.
        no_axis : bool, optional
            Whether to show the axis. Default is False.
        tight : bool, optional
            Whether to show the axis tightly. Default is True.

    '''


    def __init__(self, grid_map=None, objects=[], x_range=[0, 10], y_range=[0, 10], saved_figure=dict(), saved_ani=dict(), **kwargs) -> None:

        self.fig, self.ax = plt.subplots()

        self.x_range = x_range
        self.y_range = y_range

        self.init_plot(grid_map, objects, **kwargs)
        self.color_map = {'robot': 'g', 'obstacle': 'k', 'landmark': 'b', 'target': 'pink'}
        
        # 
        self.color_map.update(kwargs.get('color_map', dict()))

        self.saved_figure_kwargs = saved_figure
        self.saved_ani_kwargs = saved_ani
        

        self.dyna_line_list = []
        self.dyna_point_list = []


    def init_plot(self, grid_map, objects, no_axis=False, tight=True):
        
        '''
        Initialize the plot with the given grid map and objects.

        Args:
        ----
            grid_map (optional): The grid map of the environment.
            objects (list): List of objects to plot.
            no_axis (bool, optional): Whether to show the axis. Default is False.
            tight (bool, optional): Whether to show the axis tightly. Default is True.
        '''

        self.ax.set_aspect('equal') 
        self.ax.set_xlim(self.x_range) 
        self.ax.set_ylim(self.y_range)
        
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")

        self.draw_components('static', objects)
        self.draw_grid_map(grid_map)

        if no_axis: plt.axis('off')
        if tight: self.fig.tight_layout()


    def draw_components(self, mode='all', objects=[], **kwargs):

        '''
        Draw the components in the environment.

        Args:
        ----
            mode: 
                - static: draw static objects
                - dynamic: draw dynamic objects
                - all: draw all objects
            
            objects: list of objects to draw
        
        kwargs: object plot kwargs
        '''
        
        if mode == 'static':
            [obj.plot(self.ax, **kwargs) for obj in objects if obj.static]
                
        elif mode == 'dynamic':
            [obj.plot(self.ax, **kwargs) for obj in objects if not obj.static]

        elif mode == 'all':
            [obj.plot(self.ax, **kwargs) for obj in objects]
        else:
            logging.error('error input of the draw mode')
    
    def clear_components(self, mode='all', objects=[]):

        '''
        Clear the components in the environment.

        Args:
        ---- 
            mode:
                - static: clear static objects
                - dynamic: clear dynamic objects
                - all: clear all objects
            
            objects: list of objects to clear
        '''

        if mode == 'dynamic':
            [obj.plot_clear() for obj in objects if not obj.static]
            [line.pop(0).remove() for line in self.dyna_line_list]
            [points.remove() for points in self.dyna_point_list]

            self.dyna_line_list = []
            self.dyna_point_list = []

        elif mode == 'static':
            pass

        elif mode == 'all':

            if objects:
                [obj.plot_clear() for obj in objects]
            else:
                plt.cla()
    

    def draw_grid_map(self, grid_map=None, **kwargs):
        
        '''
        Draw the grid map on the plot
        '''

        if grid_map is not None:
            self.ax.imshow(grid_map.T, cmap='Greys', origin='lower', extent = self.x_range + self.y_range) 


    def draw_geometry(self, geometry, **kwargs):
        
        if geometry.geom_type == 'Point':
            pass

        elif geometry.geom_type == 'MultiLineString':

            lines = geometry.coords.xy

    def draw_trajectory(self, traj, traj_type='g-', label='trajectory', show_direction=False, refresh=False, **kwargs):

        '''
        Draw a trajectory on the plot

        Args:
        ----
            traj: list of points, point: 3*1 np.array, [x, y, theta]
            traj_type: trajectory type, default is 'g-', see https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.plot.html for detail
            label: label of the trajectory
            show_direction: whether to show the direction of the trajectory
            refresh: whether to refresh the plot (clear the previous trajectory)
            kwargs: other arguments for the ax plot, see https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.plot.html for detail
        '''

        # traj: a list of points
        if isinstance(traj, list):
            path_x_list = [p[0, 0] for p in traj]
            path_y_list = [p[1, 0] for p in traj]
        elif isinstance(traj, np.ndarray):
            # raw*column: points * num
            path_x_list = [p[0] for p in traj.T]
            path_y_list = [p[1] for p in traj.T]

        
        line = self.ax.plot(path_x_list, path_y_list, traj_type, label=label, **kwargs)

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


    def draw_points(self, point_list, s=10, c='m', refresh=True, **kwargs):

        '''
        Draw points on the plot

        Args:
        ----
            point_list: list of points, point: 2*1 np.array or [x, y]
            s: size of the points
            c: color of the points
            refresh: whether to refresh the plot (clear the previous points)
            kwargs: other arguments for the ax scatter, see https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.scatter.html for detail

        '''

        if point_list is not None:

            x_coordinates = [point[0] for point in point_list]
            y_coordinates = [point[1] for point in point_list]

            points = self.ax.scatter(x_coordinates, y_coordinates, s, c, **kwargs)

            if refresh: self.dyna_point_list.append(points)


    def draw_box(self, vertices, refresh=False, color='b-'):

        '''
        Draw a box by the vertices

        Args:
        ----
            vertices: 2*edge_number np.array, vertices of the box
            refresh: whether to refresh the plot (clear the previous box)
            color: color and line type of the box
        '''

        temp_vertex = np.c_[vertices, vertices[0:2, 0]]         
        box_line = self.ax.plot(temp_vertex[0, :], temp_vertex[1, :], color)

        if refresh: 
            self.dyna_line_list.append(box_line)

    
    def save_gif_figure(self, format='png', **kwargs):

        '''
        save the figure for generating animation

        Args:
        ----
            format: format of the figure, default is 'png'
            kwargs: other arguments for the ax savefig, see https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.savefig.html for detail, default is {'dpi': 100, 'bbox_inches': 'tight'}       
        '''

        fp = pm.ani_buffer_path
        
        if not os.path.exists(fp): os.makedirs(fp)  

        order = str(world_param.count).zfill(3)

        self.saved_figure_kwargs.update({'dpi': 100, 'bbox_inches': 'tight'})
        self.saved_figure_kwargs.update(kwargs)

        self.fig.savefig(fp+'/'+order+'.'+format, format=format, **self.saved_figure_kwargs)


    def save_animate(self, ani_name='animation', suffix='.gif', keep_len=30, rm_fig_path=True, **kwargs):
        
        '''
        Save the animation

        Args:
        ----
            ani_name: name of the animation, default is 'animation'
            suffix: suffix of the animation, default is '.gif'
            keep_len: length of the last frame, default is 30
            rm_fig_path: whether to remove the figure path after saving the animation, default is True
            kwargs: other arguments for the imageio.mimsave, see https://imageio.readthedocs.io/en/v2.8.0/format_gif-pil.html
        '''

        self.saved_ani_kwargs.update({'subrectangles': True})
        self.saved_ani_kwargs.update(kwargs)

        env_param.logger.info('Start to create animation')
        
        ap = pm.ani_path
        fp = pm.ani_buffer_path

        if not os.path.exists(ap): os.makedirs(ap)  
     
        images = list(glob.glob(fp + '/*.png'))

        images.sort()
        image_list = []
        for i, file_name in enumerate(images):

            if i == 0: continue

            image_list.append(imageio.imread(str(file_name)))
            if i == len(images) - 1:
                for j in range(keep_len):
                    image_list.append(imageio.imread(str(file_name)))

        imageio.mimsave(ap +'/'+ ani_name + suffix, image_list, **self.saved_ani_kwargs)

        env_param.logger.info('Create animation successfully, saved in the path ' + ap)
 
        if rm_fig_path: shutil.rmtree(fp)

        
    def show(self):

        '''
        Show the plot
        '''

        plt.show()



        


    