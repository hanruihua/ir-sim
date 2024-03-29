from math import inf, pi
from ir_sim.world import ObjectBase
from ir_sim.world.robots.robot_diff import RobotDiff
from ir_sim.util.util import diff_to_omni

class ObstacleDiff(RobotDiff):
    def __init__(self, shape: str = 'circle', shape_tuple=(0, 0, 0.2), color='k', **kwargs):
        super(ObstacleDiff, self).__init__(shape=shape, shape_tuple=shape_tuple, color=color, **kwargs)

        self.role='obstacle'



    def plot(self, ax, **kwargs):
        super().plot(ax, **kwargs)


    @property
    def velocity_xy(self):
        return diff_to_omni(self.state[2, 0], self._velocity)




    