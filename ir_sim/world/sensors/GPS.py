import numpy as np

class GPS:
    def __init__(self, robot_state, noise=[0.1, 0.1], **kwargs) -> None:
        self.noise = np.c_[noise]
        self.gps_list = []
        self.gps = robot_state

    def step(self, robot_state=np.zeros((3, 1))):
        self.gps = robot_state[0:2] + np.random.normal(loc=np.zeros((2, 1)), scale=self.noise)
        self.gps_list.append(self.gps)

    def plot(self, ax, color='c', markersize=5, refresh=True, **kwargs):
        
        plot_patch_list = []
        plot_line_list = []

        gps_point = ax.plot(self.gps[0, 0], self.gps[1, 0], marker="o", markersize=markersize, markerfacecolor=color, markeredgecolor=color, **kwargs)

        gps_point[0].set_zorder(4)

        if refresh:
            plot_line_list.append(gps_point)
        
        return plot_line_list, plot_patch_list

    
        