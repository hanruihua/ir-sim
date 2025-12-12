import yaml
import matplotlib.pyplot as plt
import numpy as np
from entities import Robot, Obstacle

class World:
    def __init__(self, width=10.0, height=10.0, step_time=0.1):
        self.width = width
        self.height = height
        self.dt = step_time
        self.time = 0.0
        
        self.robots = []
        self.obstacles = []
        
        # UI
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.ax.set_xlim(0, self.width)
        self.ax.set_ylim(0, self.height)
        self.ax.set_aspect('equal')
        self.ax.grid(True)

    @classmethod
    def from_yaml(cls, yaml_path):
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
            
        world_cfg = data.get('world', {})
        width = world_cfg.get('width', 10.0)
        height = world_cfg.get('height', 10.0)
        step_time = world_cfg.get('step_time', 0.1)
        
        world = cls(width=width, height=height, step_time=step_time)
        
        # Load Obstacles
        obs_list = data.get('obstacles', [])
        for obs_data in obs_list:
            x = obs_data.get('x', 0)
            y = obs_data.get('y', 0)
            radius = obs_data.get('radius', 0.5)
            shape = obs_data.get('shape', 'circle')
            width = obs_data.get('width', 1.0)
            height = obs_data.get('height', 1.0)
            theta = obs_data.get('theta', 0.0)
            
            obs = Obstacle(x=x, y=y, radius=radius, shape=shape, 
                           width=width, height=height, theta=theta)
            world.add_obstacle(obs)
            
        # Load Robots
        # (For now, we just create the entities, controller linking happens in main)
        # We need a way to return robot configs to main, or main parses them.
        # Let's simple return world and raw data
        return world, data
        
    def add_robot(self, robot):
        self.robots.append(robot)
        self.robot = robot # Keep reference to last added (or main) for legacy

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

    def step(self):
        """
        Advance one simulation step
        """
        self.time += self.dt
        
        # Update All Robots
        for bot in self.robots:
            bot.step(self.dt)
            # Simple boundary check
            margin = 0.01
            bot.state[0] = np.clip(bot.state[0], 0, self.width - margin)
            bot.state[1] = np.clip(bot.state[1], 0, self.height - margin)

    def render(self):
        self.ax.cla() # Clear axis
        self.ax.set_xlim(0, self.width)
        self.ax.set_ylim(0, self.height)
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.ax.set_title(f"Time: {self.time:.2f} s")
        
        # Render Obstacles
        for obs in self.obstacles:
            obs.render(self.ax)
            
        # Render Robot
        if self.robot:
            self.robot.render(self.ax)
            
        plt.draw()
        plt.pause(0.01)
