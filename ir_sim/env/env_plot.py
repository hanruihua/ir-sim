import matplotlib.pyplot as plt
import logging
from ir_sim.global_param.path_param import path_manager as pm
from ir_sim.global_param import world_param, env_param
import os
import imageio
import shutil
import glob
from math import sin, cos
import numpy as np


class EnvPlot:

    def __init__(self, grid_map=None, objects=[], x_range=[0, 10], y_range=[0, 10], subplot=False, saved_figure=dict(), saved_ani=dict(), **kwargs) -> None:


        '''

        saved_figure: kwargs for saving the figure, see https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.savefig.html for detail
        saved_ani: kwargs for saving the animation, see https://imageio.readthedocs.io/en/v2.8.0/format_gif-pil.html#gif-pil for detail


        kwargs:
            color_map: color map for different objects
            no_axis (default False): whether to show the axis. 
            tight (default True): whether to show the axis tightly
        '''



        if not subplot:
            self.fig, self.ax = plt.subplots()

        # else:
        #     self.fig, self.ax, self.sub_ax_list = self.sub_world_plot()

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
        # mode: static, dynamic, all
        
        if mode == 'static':
            [obj.plot(self.ax, **kwargs) for obj in objects if obj.static]
                
        elif mode == 'dynamic':
            [obj.plot(self.ax, **kwargs) for obj in objects if not obj.static]

        elif mode == 'all':
            [obj.plot(self.ax, **kwargs) for obj in objects]
        else:
            logging.error('error input of the draw mode')
    
    def clear_components(self, mode='all', objects=[], **kwargs):

        if mode == 'dynamic':
            [obj.plot_clear() for obj in objects if not obj.static]
            [line.pop(0).remove() for line in self.dyna_line_list]
            [points.remove() for points in self.dyna_point_list]

            self.dyna_line_list = []
            self.dyna_point_list = []

        elif mode == 'static':
            pass

        elif mode == 'all':
            plt.cla()
    

    def draw_grid_map(self, grid_map=None, **kwargs):
        
        if grid_map is not None:
            self.ax.imshow(grid_map.T, cmap='Greys', origin='lower', extent = self.x_range + self.y_range) 


    def draw_geometry(self, geometry, **kwargs):
        
        if geometry.geom_type == 'Point':
            pass

        elif geometry.geom_type == 'MultiLineString':

            lines = geometry.coords.xy

    def draw_trajectory(self, traj, traj_type='g-', label='trajectory', show_direction=False, refresh=False, **kwargs):
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

        if point_list is not None:

            x_coordinates = [point[0] for point in point_list]
            y_coordinates = [point[1] for point in point_list]

            points = self.ax.scatter(x_coordinates, y_coordinates, s, c, **kwargs)

            if refresh: self.dyna_point_list.append(points)


    def draw_box(self, vertices, refresh=False, color='b-', **kwargs):
        # draw a box by the vertices
        # vertices: 2*4, 2*8, 2*12, 2*16
        temp_vertex = np.c_[vertices, vertices[0:2, 0]]         
        box_line = self.ax.plot(temp_vertex[0, :], temp_vertex[1, :], color)

        if refresh: 
            self.dyna_line_list.append(box_line)

        


    # save animation and figure 
    def save_gif_figure(self, format='png', **kwargs):

        # saved_figure_kwargs: arguments when saving the figures for animation, see https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.savefig.html for detail

        fp = pm.ani_buffer_path
        
        if not os.path.exists(fp): os.makedirs(fp)  

        order = str(world_param.count).zfill(3)

        self.saved_figure_kwargs.update({'dpi': 300, 'bbox_inches': 'tight'})
        self.saved_figure_kwargs.update(kwargs)

        self.fig.savefig(fp+'/'+order+'.'+format, format=format, **self.saved_figure_kwargs)


    def save_animate(self, ani_name='animation', suffix='.gif', keep_len=30, rm_fig_path=True, **kwargs):
        
        # saved_ani_kwargs: arguments for animations(gif): see https://imageio.readthedocs.io/en/v2.8.0/format_gif-pil.html#gif-pil for detail

        self.saved_ani_kwargs.update({'subrectangles': True})
        self.saved_ani_kwargs.update(kwargs)

        # ani_name = self.saved_ani_kwargs.get('ani_name', 'animation')
        # suffix = self.saved_ani_kwargs.get('suffix', '.gif')
        # keep_len = self.saved_ani_kwargs.get('keep_len', 30)
        # rm_fig_path = self.saved_ani_kwargs.get('rm_fig_path', True)

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
        plt.show()


    # def sub_world_plot(self):

    #     # row: default 3
    #     # colum: default 3
    #     # number: number of subplot
    #     # scheme: default; 
    #     #         custom; 
    #     # custom_layout: coordinate of the main and sub axises (custom scheme)
    #     #       - [[x_min, x_max, y_min, y_max], [x_min, x_max, y_min, y_max]]
    #     #   
    #     number = self.sub_plot_kwargs.get('number', 0)
    #     row = self.sub_plot_kwargs.get('row', 3)
    #     column = self.sub_plot_kwargs.get('column', 3)
    #     layout = self.sub_plot_kwargs.get('layout', 'default')
    #     custom_layout = self.sub_plot_kwargs.get('custom_layout', [])

    #     fig = plt.figure(constrained_layout=True)
    #     sub_ax_list = []

    #     assert number < row * column

    #     if number == 0:
    #         ax = fig.add_subplot(111)
    #     else:
    #         gs = GridSpec(row, column, figure=fig)

    #         if layout == 'default':
    #             coordinate = [[0, row, 0, column-1], [0, 1, 2, 3], [1, 2, 2, 3], [2, 3, 2, 3]]
    #         elif layout == 'custom':
    #             coordinate = custom_layout

    #         for n in range(number): 
    #             c = coordinate[n]

    #             if n == 0:
    #                 ax = fig.add_subplot(gs[c[0]:c[1], c[2]:c[3]])
    #             else:
    #                 sub_ax = fig.add_subplot(gs[c[0]:c[1], c[2]:c[3]])
    #                 self.init_sub_plot(sub_ax)
    #                 sub_ax_list.append(sub_ax) 

    #     return fig, ax, sub_ax_list

        


    
