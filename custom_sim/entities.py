import numpy as np
from utils import wrap_to_pi
import matplotlib.pyplot as plt
class Entity:
    def __init__(self, x, y, radius=0.2, color='k'):
        self.state = np.array([x, y], dtype=float) # Position x, y
        self.radius = radius
        self.color = color

    def render(self, ax):
        # To be implemented by subclasses or generic circle
        circle = plt.Circle(self.state, self.radius, color=self.color)
        ax.add_patch(circle)

class Obstacle(Entity):
    def __init__(self, x, y, radius=0.5, shape='circle', width=1.0, height=1.0, theta=0.0):
        super().__init__(x, y, radius, color='gray')
        self.shape = shape
        self.width = width
        self.height = height
        self.theta = theta # Orientation for rectangle
        self.id = -1 

    def render(self, ax):
        if self.shape == 'circle':
            circle = plt.Circle(self.state, self.radius, color=self.color, alpha=0.5)
            ax.add_patch(circle)
        elif self.shape == 'rectangle':
            # Matplotlib Rectangle is defined by bottom-left corner
            # calculate bottom-left from center (x,y)
            # Assuming aligned with axes for now, or use Polygon for rotation
            # Let's support rotation
            import matplotlib.patches as patches
            import matplotlib.transforms as transforms
            
            x, y = self.state
            w, h = self.width, self.height
            # Bottom-left before rotation
            rect = patches.Rectangle((x - w/2, y - h/2), w, h, color=self.color, alpha=0.5)
            
            # Rotation transform
            t = transforms.Affine2D().rotate_around(x, y, self.theta) + ax.transData
            rect.set_transform(t)
            ax.add_patch(rect)

class Robot(Entity):
    def __init__(self, x=0, y=0, theta=0, radius=0.2, goal=None, kinematics='diff'):
        super().__init__(x, y, radius, color='b')
        self.kinematics = kinematics
        self.state = np.array([x, y, theta], dtype=float)
        self.goal = np.array(goal) if goal else None
        
        # Kinematics Params
        self.v_max = 1.0
        self.w_max = 2.0  # limit for diff drive
        self.phi_max = 0.6 # limit for ackermann (steering angle, rad)
        self.L = 0.3      # wheelbase for ackermann
        
        # Control inputs
        self.v = 0.0
        self.w = 0.0 # yaw rate (diff) or steering angle phi (ackermann)
        
        self.lidar = None

    def set_lidar(self, lidar):
        self.lidar = lidar

    def step(self, dt):
        # Update State based on kinematics
        theta = self.state[2]
        
        if self.kinematics == 'diff':
            # Differential Drive
            # x_dot = v cos(theta)
            # y_dot = v sin(theta)
            # theta_dot = w
            self.state[0] += self.v * np.cos(theta) * dt
            self.state[1] += self.v * np.sin(theta) * dt
            self.state[2] += self.w * dt
            
        elif self.kinematics == 'ackermann':
            # Ackermann Steering (Car-like)
            # v = speed, w = steering angle (phi)
            # x_dot = v cos(theta)
            # y_dot = v sin(theta)
            # theta_dot = v/L tan(phi)
            phi = self.w
            self.state[0] += self.v * np.cos(theta) * dt
            self.state[1] += self.v * np.sin(theta) * dt
            self.state[2] += (self.v / self.L) * np.tan(phi) * dt
            
        self.state[2] = wrap_to_pi(self.state[2])
        
        # Update Lidar if attached
        # if self.lidar:
        #    self.lidar.step(self.state)

    def set_velocity(self, v, w):
        self.v = np.clip(v, -self.v_max, self.v_max)
        
        if self.kinematics == 'diff':
            self.w = np.clip(w, -self.w_max, self.w_max)
        elif self.kinematics == 'ackermann':
            # Convert desired yaw rate w to steering angle phi
            # w = (v/L) * tan(phi)  =>  phi = arctan(w * L / v)
            # Avoid division by zero
            if abs(v) > 0.01:
                phi = np.arctan(w * self.L / v)
                self.w = np.clip(phi, -self.phi_max, self.phi_max) # store phi in self.w
            else:
                self.w = 0.0 # Cannot steer if not moving
