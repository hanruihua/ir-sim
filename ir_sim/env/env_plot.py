import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import logging

class EnvPlot:

    def __init__(self, grid_map=None, objects=[], x_range=[0, 10], y_range=[0, 10], subplot=False,  **kwargs) -> None:

        if not subplot:
            self.fig, self.ax = plt.subplots()

        # else:
        #     self.fig, self.ax, self.sub_ax_list = self.sub_world_plot()

        self.x_range = x_range
        self.y_range = y_range


        self.init_plot(grid_map, objects, **kwargs)
        self.corlor_map = {'robot': 'g', 'obstacle': 'k', 'landmark': 'b', 'target': 'pink'}
        

        # 
        self.corlor_map.update(kwargs.get('corlor_map', dict()))


    def init_plot(self, grid_map, objects, no_axis=False):
        
        self.ax.set_aspect('equal') 
        self.ax.set_xlim(self.x_range) 
        self.ax.set_ylim(self.y_range)
        
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")

        self.draw_components('static', objects)
        self.draw_grid_map(grid_map)

        if no_axis: plt.axis('off')


    def draw_components(self, mode='all', objects=[], **kwargs):
        # mode: static, dynamic, all
        if mode == 'static':
            [obj.plot(self.ax) for obj in objects if obj.static]
                
        elif mode == 'dynamic':
            [obj.plot(self.ax) for obj in objects if not obj.static]

        elif mode == 'all':
            [obj.plot(self.ax) for obj in objects]
        else:
            logging.error('error input of the draw mode')


    def draw_grid_map(self, grid_map=None, **kwargs):
        
        if grid_map is not None:
            self.ax.imshow(grid_map.T, cmap='Greys', origin='lower', extent = self.x_range + self.y_range) 



    def sub_world_plot(self):

        # row: default 3
        # colum: default 3
        # number: number of subplot
        # scheme: default; 
        #         custom; 
        # custom_layout: coordinate of the main and sub axises (custom scheme)
        #       - [[x_min, x_max, y_min, y_max], [x_min, x_max, y_min, y_max]]
        #   
        number = self.sub_plot_kwargs.get('number', 0)
        row = self.sub_plot_kwargs.get('row', 3)
        column = self.sub_plot_kwargs.get('column', 3)
        layout = self.sub_plot_kwargs.get('layout', 'default')
        custom_layout = self.sub_plot_kwargs.get('custom_layout', [])

        fig = plt.figure(constrained_layout=True)
        sub_ax_list = []

        assert number < row * column

        if number == 0:
            ax = fig.add_subplot(111)
        else:
            gs = GridSpec(row, column, figure=fig)

            if layout == 'default':
                coordinate = [[0, row, 0, column-1], [0, 1, 2, 3], [1, 2, 2, 3], [2, 3, 2, 3]]
            elif layout == 'custom':
                coordinate = custom_layout

            for n in range(number): 
                c = coordinate[n]

                if n == 0:
                    ax = fig.add_subplot(gs[c[0]:c[1], c[2]:c[3]])
                else:
                    sub_ax = fig.add_subplot(gs[c[0]:c[1], c[2]:c[3]])
                    self.init_sub_plot(sub_ax)
                    sub_ax_list.append(sub_ax) 

        return fig, ax, sub_ax_list

        


    
