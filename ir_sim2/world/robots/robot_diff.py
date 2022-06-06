import robot_base
import numpy as np

class robot_diff(robot_base):
    def __init__(self, id, shape='circle', step_time=0.1, radius=0.2, radius_exp=0.1, **kwargs):
        super(robot_diff, self).__init__(id=id, shape=shape, robot_type='diff', step_time=step_time, **kwargs)

        self.radius = radius
        self.radius_collision = radius + radius_exp
        self.vel_omni = np.zeros(self.vel_dim)

    def dynamics(self, vel, vel_type='diff', noise=False, alpha = [0.01, 0, 0, 0.01, 0, 0]):
        # The differential-wheel robot dynamics
        # reference: Probability robotics, motion model
        # vel_tpe: 'diff' or 'omni'
        if vel_type == 'omni':
            pass
        
        self.state = self.motion_diff(self.state, vel, step_time, noise, alpha)
        self.vel = vel
        # self.vel_omni = 

    # reference: probabilistic robotics[book], motion model P127
    def motion_diff(self, current_state, vel, sampletime, noise = False, alpha = [0.01, 0, 0, 0.01, 0, 0]):

        # vel: np.array([[linear vel],[angular vel]])
        # current_state: np.array([[x], [y], [theta]])
        # alpha: control noise includes linear, angular, orientation

        if noise == True:

            std_linear = np.sqrt(alpha[0] * (vel[0, 0] ** 2) + alpha[1] * (vel[1, 0] ** 2))
            std_angular = np.sqrt(alpha[2] * (vel[0, 0] ** 2) + alpha[3] * (vel[1, 0] ** 2))
            # gamma = alpha[4] * (vel[0, 0] ** 2) + alpha[5] * (vel[1, 0] ** 2)
            
            vel_noise = vel + np.random.normal([[0], [0]], scale = [[std_linear], [std_angular]])

        else:
            vel_noise = vel

        vt = float(vel_noise[0, 0])
        omegat = float(vel_noise[1,0])

        theta = float(wraptopi(current_state[2, 0]))

        if omegat >= 0.01 or omegat <= -0.01:
            ratio = vt/omegat
            next_state = current_state + np.array([ [-ratio * sin(theta) + ratio * sin(theta + omegat * sampletime)], 
                                        [ratio * cos(theta) - ratio * cos(theta + omegat * sampletime)], 
                                        [omegat * sampletime]])

        else:
            next_state = current_state + np.array([[vt * sampletime * cos(theta)], [vt * sampletime * sin(theta)], [0]])

        next_state[2, 0] =  float(wraptopi(next_state[2, 0])) 
        
        return next_state 

    def diff_to_omni(self,):
        pass

    def omni_to_diff(self,):
        pass


        
    
