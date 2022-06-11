import matplotlib.pyplot as plt

class EnvPlot:
    def __init__(self, width=10, height=10, components=dict(), full=False, keep_path=False, offset_x = 0, offset_y=0, **kwargs):
        
        # kwargs: for init plot, arguments in the robot plot and obstacle plot
        #         robot_plot: robot_color = 'g', goal_color='r', show_lidar=True, show_goal=False, show_text=True, show_traj=False, traj_type='-g', fontsize=10
        #         obstacle_plot: 
        self.fig, self.ax = plt.subplots()
        self.width = width
        self.height = height
        self.offset_x = offset_x
        self.offset_y = offset_y

        self.color_list = ['g', 'b', 'r', 'c', 'm', 'y', 'k', 'w']
        self.components = components
        self.keep_path=keep_path 

        self.init_plot(**kwargs)

    def init_plot(self, **kwargs):
        self.ax.set_aspect('equal')
        self.ax.set_xlim(self.offset_x, self.offset_x + self.width)
        self.ax.set_ylim(self.offset_y, self.offset_y + self.height)

        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")

        self.draw_components(**kwargs) 

        return self.ax.patches + self.ax.texts + self.ax.artists

    def draw_components(self, **kwargs):
        for robot in self.components['robots']:
            robot.plot(**kwargs)
        # for obstacle in self.components['obstacles']:
        #     obstacle.plot(**kwargs)

    def clear_components(self, ):
        self.ax.texts.clear()

        for robot in self.components['robots']:
            robot.plot_clear(**kwargs)

    # animation method 
    def save_gif_figure(self, path, i, format='png'):
        if path.exists():
            order = str(i).zfill(3)
            plt.savefig(str(path)+'/'+order+'.'+format, format=format)
        else:
            path.mkdir()
            order = str(i).zfill(3)
            plt.savefig(str(path)+'/'+order+'.'+format, format=format)

    def create_animate(self, image_path, ani_path, ani_name='animated', keep_len=30, rm_fig_path=True):

        if not ani_path.exists():
            ani_path.mkdir()

        images = list(image_path.glob('*.png'))
        images.sort()
        image_list = []
        for i, file_name in enumerate(images):

            if i == 0:
                continue

            image_list.append(imageio.imread(file_name))
            if i == len(images) - 1:
                for j in range(keep_len):
                    image_list.append(imageio.imread(file_name))

        imageio.mimsave(str(ani_path)+'/'+ ani_name+'.gif', image_list)
        print('Create animation successfully')

        if rm_fig_path:
            shutil.rmtree(image_path)


    
    



