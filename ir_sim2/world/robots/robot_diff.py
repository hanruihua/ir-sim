import robot_base

class robot_diff(robot_base):
    def __init__(self, id, shape='circle', step_time=0.1, radius=0.2, radius_exp=0.1, **kwargs):
        super(robot_diff, self).__init__(id=id, shape=shape, robot_type='diff', step_time=step_time, **kwargs)

        self.radius = radius
        self.radius_collision = radius + radius_exp
        self.vel_omni = np.zeros(self.vel_dim)

    def dynamics(self, vel, vel_type='diff'):
        # The differential-wheel robot dynamics
        # reference: Probability robotics, motion model
        # vel_tpe: 'diff' or 'omni'
        if vel_type == 'omni':
            pass

        



    def diff_to_omni(self,):
        pass

    def omni_to_diff(self,):
        pass


        
    
