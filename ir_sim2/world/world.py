import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

class world:
    def __init__(self, height=10, width=10, step_time=0.01, sample_time=0.1, offset=[0, 0], **kwargs) -> None:

        self.height = height
        self.width = width
        self.step_time = step_time
        self.sample_time = sample_time
        self.offset = offset
        self.count = 0
        self.sampling = True
        self.x_range = [self.offset[0], self.offset[0] + self.width]
        self.y_range = [self.offset[1], self.offset[1] + self.height]

        self.sub_plot_kwargs = kwargs.get('subplot', {})
        
    def step(self):
        self.count += 1
        self.sampling = (self.count % (self.sample_time / self.step_time) == 0)
        
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
    
    def init_sub_plot(self, ax):
        
        ax.set_aspect('equal') 
        ax.set_xlim(self.x_range) 
        ax.set_ylim(self.y_range)


        


