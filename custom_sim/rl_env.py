"""
Gymnasium Environment for LiDAR-based Robot Navigation

This module provides a gym.Env wrapper around the existing World, Robot, and Lidar classes
to enable reinforcement learning training for robot navigation tasks.
"""

import numpy as np
import gymnasium as gym
from gymnasium import spaces
import yaml
from typing import Tuple, Dict, Any, Optional

from world import World
from entities import Robot, Obstacle
from sensor import Lidar2D
from utils import distance, wrap_to_pi


class LidarNavigationEnv(gym.Env):
    """
    Gymnasium environment for LiDAR-based robot navigation.
    
    Observation Space:
        - LiDAR scan: 360 float values (ranges normalized to [0,1])
        - Robot state: [x, y, theta, goal_distance, goal_angle] (5 floats)
    
    Action Space:
        - [linear_velocity, angular_velocity] (2 floats, normalized)
    
    Rewards:
        - +100 for reaching goal
        - -50 for collision
        - -0.1 per time step
        - +1 for moving closer to goal
        - -1 for moving away from goal
    """
    
    metadata = {'render_modes': ['human', 'rgb_array'], 'render_fps': 30}
    
    def __init__(self, 
                 config_file: str = 'robot_world.yaml',
                 max_steps: int = 1000,
                 goal_threshold: float = 0.5,
                 collision_threshold: float = 0.3,
                 max_linear_vel: float = 1.0,
                 max_angular_vel: float = 1.0,
                 lidar_num_rays: int = 360,
                 lidar_max_range: float = 5.0,
                 render_mode: str = 'human'):
        
        super().__init__()
        
        self.config_file = config_file
        self.max_steps = max_steps
        self.goal_threshold = goal_threshold
        self.collision_threshold = collision_threshold
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.lidar_num_rays = lidar_num_rays
        self.lidar_max_range = lidar_max_range
        self.render_mode = render_mode
        
        # Environment state
        self.world = None
        self.robot = None
        self.lidar = None
        self.goal_position = None
        self.step_count = 0
        self.last_distance_to_goal = None
        self.episode_reward = 0.0
        
        # Spaces
        self._setup_spaces()
        
        # Initialize environment
        self.reset()
    
    def _setup_spaces(self):
        """Setup observation and action spaces."""
        # Observation space: [lidar_ranges(360) + robot_state(5)]
        lidar_space = spaces.Box(low=0.0, high=1.0, 
                                shape=(self.lidar_num_rays,), dtype=np.float32)
        
        # Robot state: [x, y, sin, cos, goal_distance, goal_angle]
        robot_state_space = spaces.Box(low=-np.inf, high=np.inf, 
                                      shape=(6,), dtype=np.float32)
        
        self.observation_space = spaces.Dict({
            'lidar': lidar_space,
            'robot_state': robot_state_space
        })
        
        # Action space: [linear_velocity, angular_velocity]
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0]), 
            high=np.array([1.0, 1.0]), 
            dtype=np.float32
        )
    
    def reset(self, seed: Optional[int] = None, options: Optional[Dict] = None):
        """Reset the environment to initial state."""
        super().reset(seed=seed)
        
        # Load world from YAML config
        self.world, config_data = World.from_yaml(self.config_file)
        
        # Setup robot (use first robot from config)
        if 'robots' in config_data and config_data['robots']:
            r_conf = config_data['robots'][0]
            state = r_conf.get('state', [1, 1, 0])
            goal = r_conf.get('goal', [8, 8])
            kinematics = r_conf.get('kinematics', {}).get('name', 'diff')
            radius = r_conf.get('shape', {}).get('radius', 0.2)
            
            self.robot = Robot(x=state[0], y=state[1], theta=state[2], 
                             radius=radius, goal=goal, kinematics=kinematics)
            self.goal_position = np.array(goal)
            self.world.add_robot(self.robot)
        else:
            # Default robot setup
            self.robot = Robot(x=1.0, y=1.0, theta=0.0, radius=0.2, 
                             goal=[8.0, 8.0], kinematics='diff')
            self.goal_position = np.array([8.0, 8.0])
            self.world.add_robot(self.robot)
        
        # Setup LiDAR
        self.lidar = Lidar2D(self.robot, max_range=self.lidar_max_range, 
                           num_beams=self.lidar_num_rays)
        self.robot.set_lidar(self.lidar)
        
        # Reset episode state
        self.step_count = 0
        self.last_distance_to_goal = self._get_distance_to_goal()
        self.episode_reward = 0.0
        
        return self._get_observation(), self._get_info()
    
    def step(self, action: np.ndarray) -> Tuple[Dict, float, bool, bool, Dict]:
        """
        Execute one environment step.
        
        Args:
            action: [linear_velocity, angular_velocity] normalized to [-1, 1]
            
        Returns:
            observation, reward, terminated, truncated, info
        """
        self.step_count += 1
        
        # Convert normalized action to robot velocities
        linear_vel = action[0] * self.max_linear_vel
        angular_vel = action[1] * self.max_angular_vel
        
        # Apply action to robot
        self.robot.set_velocity(linear_vel, angular_vel)
        
        # Step world physics
        self.world.step()
        
        # Update LiDAR
        lidar_ranges = self.lidar.step(self.world.obstacles)
        
        # Calculate reward
        reward = self._calculate_reward()
        self.episode_reward += reward
        
        # Check termination conditions
        terminated = self._is_goal_reached() or self._is_collision()
        truncated = self.step_count >= self.max_steps
        
        return (self._get_observation(), reward, terminated, truncated, self._get_info())
    
    def _get_observation(self) -> Dict[str, np.ndarray]:
        """Get current environment observation."""
        # Get LiDAR data
        if hasattr(self.lidar, 'last_scan') and self.lidar.last_scan is not None:
            lidar_ranges = np.array(self.lidar.last_scan, dtype=np.float32)
        else:
            # Fallback: get fresh scan
            lidar_ranges = np.array(self.lidar.step(self.world.obstacles), dtype=np.float32)
        
        # Normalize LiDAR ranges to [0, 1]
        lidar_ranges = np.clip(lidar_ranges / self.lidar_max_range, 0.0, 1.0)
        
        # Ensure correct number of rays
        if len(lidar_ranges) != self.lidar_num_rays:
            # Resize array if needed
            new_ranges = np.linspace(0, len(lidar_ranges)-1, self.lidar_num_rays, dtype=int)
            lidar_ranges = lidar_ranges[new_ranges]
        
        # Get robot state
        robot_x, robot_y, robot_theta = self.robot.state[:3]
        goal_distance = self._get_distance_to_goal()
        goal_angle = self._get_angle_to_goal()
        
        robot_state = np.array([
            robot_x / self.world.width,  # Normalize position
            robot_y / self.world.height,
            np.sin(robot_theta),  # Encode angle as sin/cos for continuity
            np.cos(robot_theta),
            goal_distance / np.sqrt(self.world.width**2 + self.world.height**2),  # Normalized
            goal_angle / np.pi  # Normalized angle to goal
        ], dtype=np.float32)
        
        return {
            'lidar': lidar_ranges,
            'robot_state': robot_state
        }
    
    def _calculate_reward(self) -> float:
        """Calculate reward for current state."""
        reward = 0.0
        
        # Goal reached reward
        if self._is_goal_reached():
            reward += 100.0
            
        # Collision penalty
        elif self._is_collision():
            reward -= 50.0
            
        # Time penalty (encourage efficiency)
        reward -= 0.1
        
        # Distance-based reward (encourage progress toward goal)
        current_distance = self._get_distance_to_goal()
        if self.last_distance_to_goal is not None:
            distance_reward = (self.last_distance_to_goal - current_distance) * 10.0
            reward += distance_reward
        
        self.last_distance_to_goal = current_distance
        
        # Penalty for being too close to obstacles (encourage safe navigation)
        min_obstacle_distance = self._get_min_obstacle_distance()
        if min_obstacle_distance < 0.8:  # Safety margin
            reward -= (0.8 - min_obstacle_distance) * 5.0
            
        return reward
    
    def _is_goal_reached(self) -> bool:
        """Check if robot has reached the goal."""
        return self._get_distance_to_goal() < self.goal_threshold
    
    def _is_collision(self) -> bool:
        """Check if robot has collided with obstacles."""
        robot_pos = np.array(self.robot.state[:2])
        robot_radius = self.robot.radius
        
        for obstacle in self.world.obstacles:
            if obstacle.shape == 'circle':
                obstacle_pos = np.array(obstacle.state[:2])
                obstacle_radius = obstacle.radius
                
                dist = distance(robot_pos, obstacle_pos)
                if dist < (robot_radius + obstacle_radius + self.collision_threshold):
                    return True
            
            elif obstacle.shape == 'rectangle':
                # Circle-Rectangle Collision (Rotated)
                # 1. Transform Robot center to Obstacle local frame
                ox, oy = obstacle.state[:2]
                w, h = obstacle.width, obstacle.height
                theta = obstacle.theta
                
                dx = robot_pos[0] - ox
                dy = robot_pos[1] - oy
                
                # Rotate by -theta
                c, s = np.cos(-theta), np.sin(-theta)
                local_x = dx * c - dy * s
                local_y = dx * s + dy * c
                
                # 2. Find closest point on Box to Circle center
                closest_x = np.clip(local_x, -w/2, w/2)
                closest_y = np.clip(local_y, -h/2, h/2)
                
                # 3. Distance check
                dist_x = local_x - closest_x
                dist_y = local_y - closest_y
                dist_sq = dist_x*dist_x + dist_y*dist_y
                
                if dist_sq < (robot_radius + self.collision_threshold)**2:
                    return True
                
        # Check world boundaries
        x, y = robot_pos
        if (x - robot_radius < 0 or x + robot_radius > self.world.width or
            y - robot_radius < 0 or y + robot_radius > self.world.height):
            return True
            
        return False
    
    def _get_distance_to_goal(self) -> float:
        """Get distance from robot to goal."""
        robot_pos = np.array(self.robot.state[:2])
        return distance(robot_pos, self.goal_position)
    
    def _get_angle_to_goal(self) -> float:
        """Get angle from robot to goal."""
        robot_pos = np.array(self.robot.state[:2])
        robot_theta = self.robot.state[2]
        
        goal_direction = self.goal_position - robot_pos
        goal_angle = np.arctan2(goal_direction[1], goal_direction[0])
        
        return wrap_to_pi(goal_angle - robot_theta)
    
    def _get_min_obstacle_distance(self) -> float:
        """Get minimum distance to any obstacle."""
        if hasattr(self.lidar, 'last_scan') and self.lidar.last_scan is not None:
            return min(self.lidar.last_scan)
        return self.lidar_max_range
    
    def _get_info(self) -> Dict[str, Any]:
        """Get additional info for debugging."""
        return {
            'step_count': self.step_count,
            'distance_to_goal': self._get_distance_to_goal(),
            'goal_reached': self._is_goal_reached(),
            'collision': self._is_collision(),
            'episode_reward': self.episode_reward,
            'robot_position': self.robot.state[:2],
            'goal_position': self.goal_position.tolist()
        }
    
    def render(self, mode: str = 'human'):
        """Render the environment."""
        if mode == 'human':
            if hasattr(self.world, 'ax'):
                self.world.ax.cla()
                self.world.ax.set_xlim(0, self.world.width)
                self.world.ax.set_ylim(0, self.world.height)
                self.world.ax.set_aspect('equal')
                self.world.ax.grid(True)
                
                # Render obstacles
                # Render obstacles
                for obs in self.world.obstacles:
                    import matplotlib.pyplot as plt
                    import matplotlib.patches as patches
                    import matplotlib.transforms as transforms
                    
                    if obs.shape == 'circle':
                        circle = plt.Circle(obs.state[:2], obs.radius, 
                                          color='gray', alpha=0.7)
                        self.world.ax.add_patch(circle)
                    elif obs.shape == 'rectangle':
                        x, y = obs.state[:2]
                        w, h = obs.width, obs.height
                        rect = patches.Rectangle((x - w/2, y - h/2), w, h, 
                                               color='gray', alpha=0.7)
                        t = transforms.Affine2D().rotate_around(x, y, obs.theta) + self.world.ax.transData
                        rect.set_transform(t)
                        self.world.ax.add_patch(rect)
                
                # Render robot
                self.robot.render(self.world.ax)
                
                # Render goal
                self.world.ax.plot(self.goal_position[0], self.goal_position[1], 
                                 'g*', markersize=15, label='Goal')
                
                # Render LiDAR rays (optional)
                if hasattr(self.lidar, 'last_scan') and self.lidar.last_scan is not None:
                    robot_x, robot_y, robot_theta = self.robot.state[:3]
                    for i, r in enumerate(self.lidar.last_scan):
                        if i % 10 == 0:  # Draw every 10th ray to avoid clutter
                            angle = robot_theta + self.lidar.angles[i]
                            end_x = robot_x + r * np.cos(angle)
                            end_y = robot_y + r * np.sin(angle)
                            self.world.ax.plot([robot_x, end_x], [robot_y, end_y], 
                                             'r-', alpha=0.3, linewidth=0.5)
                
                self.world.ax.set_title(f'RL Navigation - Step: {self.step_count}, Reward: {self.episode_reward:.2f}')
                
                import matplotlib.pyplot as plt
                plt.draw()
                plt.pause(0.01)
    
    def close(self):
        """Close the environment."""
        if hasattr(self.world, 'fig'):
            import matplotlib.pyplot as plt
            plt.close(self.world.fig)