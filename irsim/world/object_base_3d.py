from irsim.world import ObjectBase
import matplotlib as mpl
from matplotlib import transforms as mtransforms
from matplotlib import image
from irsim.global_param.path_param import path_manager
from math import pi, atan2, cos, sin
import mpl_toolkits.mplot3d.art3d as art3d
import numpy as np

class ObjectBase3D(ObjectBase):
        
        def __init__(self, **kwargs):
            super().__init__(**kwargs)


        def plot_object(self, ax, **kwargs):
            """
            Plot the object itself.

            Args:
                ax: Matplotlib axis.
                **kwargs: Additional plotting options.
            """
            if self.description is None:
                x = self.state_re[0, 0]
                y = self.state_re[1, 0]

                if self.shape == "circle":
                    object_patch = mpl.patches.Circle(
                        xy=(x, y), radius=self.radius, color=self.color
                    )
                    object_patch.set_zorder(3)
                    ax.add_patch(object_patch)

                    art3d.patch_2d_to_3d(object_patch, z=self.z)

                elif self.shape == "polygon" or self.shape == "rectangle":
                    object_patch = mpl.patches.Polygon(xy=self.vertices.T, color=self.color)
                    object_patch.set_zorder(3)
                    ax.add_patch(object_patch)

                    art3d.patch_2d_to_3d(object_patch, z=self.z)

                elif self.shape == "linestring":
                    object_patch = mpl.lines.Line2D(
                        self.vertices[0, :], self.vertices[1, :], color=self.color
                    )
                    object_patch.set_zorder(3)
                    ax.add_line(object_patch)

                elif self.shape == "map":
                    return

                self.plot_patch_list.append(object_patch)

            else:
                self.plot_object_image(ax, self.description, **kwargs)


        def plot_goal(self, ax, goal_color="r"):
            """
            Plot the goal position of the object.

            Args:
                ax: Matplotlib axis.
                goal_color (str): Color of the goal marker.
            """
            goal_x = self.goal_re[0, 0]
            goal_y = self.goal_re[1, 0]

            goal_circle = mpl.patches.Circle(
                xy=(goal_x, goal_y), radius=self.radius, color=goal_color, alpha=0.5
            )
            goal_circle.set_zorder(1)

            ax.add_patch(goal_circle)

            # 3D plot
            art3d.patch_2d_to_3d(goal_circle, z=self.z)

            self.plot_patch_list.append(goal_circle)

        def plot_arrow(self, ax, arrow_length=0.4, arrow_width=0.6, **kwargs):
            """
            Plot an arrow indicating the velocity orientation of the object.

            Args:
                ax: Matplotlib axis.
                arrow_length (float): Length of the arrow.
                arrow_width (float): Width of the arrow.
                **kwargs: Additional plotting options.
            """
            x = self.state_re[0][0]
            y = self.state_re[1][0]
            theta = atan2(self.velocity_xy[1, 0], self.velocity_xy[0, 0])
            arrow_color = kwargs.get("arrow_color", "gold")

            arrow = mpl.patches.Arrow(
                x,
                y,
                arrow_length * cos(theta),
                arrow_length * sin(theta),
                width=arrow_width,
                color=arrow_color,
            )
            arrow.set_zorder(3)
            ax.add_patch(arrow)

            # 3D plot
            art3d.patch_2d_to_3d(arrow, z=self.z)

            self.plot_patch_list.append(arrow)

        def plot_trail(self, ax, **kwargs):
            """
            Plot the trail of the object.

            Args:
                ax: Matplotlib axis.
                **kwargs: Additional plotting options.
            """
            trail_type = kwargs.get("trail_type", self.shape)
            trail_edgecolor = kwargs.get("edgecolor", self.color)
            trail_linewidth = kwargs.get("linewidth", 0.8)
            trail_alpha = kwargs.get("trail_alpha", 0.7)
            trail_fill = kwargs.get("trail_fill", False)
            trail_color = kwargs.get("trail_color", self.color)

            r_phi_ang = 180 * self._state[2, 0] / pi

            if trail_type == "rectangle" or trail_type == "polygon":
                start_x = self.vertices[0, 0]
                start_y = self.vertices[1, 0]

                car_rect = mpl.patches.Rectangle(
                    xy=(start_x, start_y),
                    width=self.length,
                    height=self.width,
                    angle=r_phi_ang,
                    edgecolor=trail_edgecolor,
                    fill=False,
                    alpha=trail_alpha,
                    linewidth=trail_linewidth,
                    facecolor=trail_color,
                )
                ax.add_patch(car_rect)

                # 3D plot
                art3d.patch_2d_to_3d(car_rect, z=self.z)

            elif trail_type == "circle":
                car_circle = mpl.patches.Circle(
                    xy=self.centroid,
                    radius=self.radius,
                    edgecolor=trail_edgecolor,
                    fill=trail_fill,
                    alpha=trail_alpha,
                    facecolor=trail_color,
                )
                ax.add_patch(car_circle)

                # 3D plot
                art3d.patch_2d_to_3d(car_circle, z=self.z)


        def plot_trajectory(self, ax, keep_length=0, **kwargs):

            """
            Plot the trajectory of the object.

            Args:
                ax: Matplotlib axis.
                keep_length (int): Number of steps to keep in the plot.
                **kwargs: Additional plotting options.
            """
            traj_color = kwargs.get("traj_color", self.color)
            traj_style = kwargs.get("traj_style", "-")
            traj_width = kwargs.get("traj_width", self.width)
            traj_alpha = kwargs.get("traj_alpha", 0.5)

            x_list = [t[0, 0] for t in self.trajectory[-keep_length:]]
            y_list = [t[1, 0] for t in self.trajectory[-keep_length:]]

            # linewidth = linewidth_from_data_units_3d(traj_width, ax, "z") 
            linewidth = traj_width * 10
            solid_capstyle = "round" if self.shape == "circle" else "butt"

            self.plot_line_list.append(
                ax.plot(
                    x_list,
                    y_list,
                    color=traj_color,
                    linestyle=traj_style,
                    linewidth=linewidth,
                    solid_joinstyle="round",
                    solid_capstyle=solid_capstyle,
                    alpha=traj_alpha,
                    zs=0, zdir='z',
                )
                
            )

        
            
            