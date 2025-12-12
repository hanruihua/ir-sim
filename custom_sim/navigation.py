import numpy as np
import heapq
from utils import distance, wrap_to_pi

class AStarPlanner:
    def __init__(self, occupancy_grid):
        self.grid = occupancy_grid
        
    def plan(self, start_pos, goal_pos):
        """
        A* Algorithm
        start_pos: (x, y)
        goal_pos: (x, y)
        Returns: list of (x, y) waypoints
        """
        start_node = self.grid.to_grid(start_pos[0], start_pos[1])
        goal_node = self.grid.to_grid(goal_pos[0], goal_pos[1])
        
        if not start_node or not goal_node:
            print(f"Start or Goal out of bounds! Start: {start_pos} -> {start_node}, Goal: {goal_pos} -> {goal_node}")
            return None
            
        open_set = []
        heapq.heappush(open_set, (0, start_node))
        
        came_from = {}
        g_score = {start_node: 0}
        f_score = {start_node: distance(np.array(start_node), np.array(goal_node))}
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if current == goal_node:
                return self.reconstruct_path(came_from, current)
                
            for neighbor in self.get_neighbors(current):
                # Check occupancy (Threshold p > 0.5 is occupied)
                if self.grid.grid_log_odds[neighbor] > 0: # >0 log odds means p > 0.5
                    continue
                    
                tentative_g_score = g_score[current] + 1 # assume grid cost 1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f = tentative_g_score + distance(np.array(neighbor), np.array(goal_node))
                    f_score[neighbor] = f
                    heapq.heappush(open_set, (f, neighbor))
                    
        print("No path found")
        return []

    def get_neighbors(self, node):
        x, y = node
        neighbors = []
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]: # 8-connectivity
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.grid.nx and 0 <= ny < self.grid.ny:
                # SIMPLE INFLATION CHECK
                # If any neighbor of (nx, ny) is occupied, consider (nx, ny) unsafe (Inflated prob)
                is_safe = True
                # Check 3x3 around neighbor
                for ix in range(-1, 2):
                    for iy in range(-1, 2):
                        chk_x, chk_y = nx + ix, ny + iy
                        if 0 <= chk_x < self.grid.nx and 0 <= chk_y < self.grid.ny:
                            if self.grid.grid_log_odds[chk_x, chk_y] > 0: # Occupied
                                is_safe = False
                                break
                    if not is_safe: break
                
                if is_safe:
                    neighbors.append((nx, ny))
        return neighbors

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        
        # Convert back to world coordinates
        world_path = []
        for ix, iy in path:
            wx = (ix + 0.5) * self.grid.resolution
            wy = (iy + 0.5) * self.grid.resolution
            world_path.append([wx, wy])
        
        return world_path

class DWAController:
    def __init__(self, robot):
        self.robot = robot
        # Parameters
        self.predict_time = 3.0 # sec
        self.dt = 0.1
        self.v_res = 0.1
        self.w_res = 0.1 # rad
        
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.max_speed = 1.0 # m/s
        self.max_yaw_rate = 100.0 * np.pi / 180.0 

    def plan(self, point_cloud, goal):
        """
        Dynamic Window Approach
        """
        dw = self.calc_dynamic_window()
        
        best_u = [0.0, 0.0]
        max_score = -float('inf')
        
        # Grid search (v, w)
        for v in np.arange(dw[0], dw[1], self.v_res):
            for w in np.arange(dw[2], dw[3], self.w_res):
                trajectory = self.predict_trajectory(v, w)
                
                # Calc Costs
                score = self.calc_score(trajectory, point_cloud, goal, v)
                
                if score > max_score:
                    max_score = score
                    best_u = [v, w]
        
        # Recovery Behavior: If all paths blocked, rotate in place
        if max_score == -float('inf'):
            print("Recovery Mode: Rotating")
            return [0.0, -self.max_yaw_rate] # Rotate right
                    
        return best_u

    def calc_dynamic_window(self):
        # [v_min, v_max, w_min, w_max]
        # In this simple sim, we assume infinite acceleration for now or large enough
        # But DWA should constrain by accel:
        # Vs = [0, max_v, -max_w, max_w]
        # Vd = [v-a*dt, v+a*dt, ...]
        
        # For simplicity, returning static limits
        return [0.0, self.max_speed, -self.max_yaw_rate, self.max_yaw_rate]

    def predict_trajectory(self, v, w):
        x, y, theta = self.robot.state
        traj = []
        time = 0
        while time <= self.predict_time:
            x += v * np.cos(theta) * self.dt
            y += v * np.sin(theta) * self.dt
            theta += w * self.dt
            traj.append([x, y])
            time += self.dt
        return np.array(traj)

    def calc_score(self, trajectory, point_cloud, goal, v):
        # 1. Heading to Goal
        dx = goal[0] - trajectory[-1][0]
        dy = goal[1] - trajectory[-1][1]
        dist_to_goal = np.hypot(dx, dy)
        goal_score = 1.0 / (dist_to_goal + 0.1)
        
        # 2. Obstacle Clearance
        min_dist = float('inf')
        
        # Check every point in trajectory against every obstacle point? -> Expensive
        # Optimization: Check only trajectory endpoints or simple sampling
        
        if len(point_cloud) > 0:
            # Let's check min dist of trajectory to nearest obstacle
            for tx, ty in trajectory:
                dists = np.hypot(point_cloud[:,0] - tx, point_cloud[:,1] - ty)
                if len(dists) > 0:
                    min_d = np.min(dists)
                    if min_d < min_dist:
                        min_dist = min_d
        
        # Safety Margin: Robot Radius
        if min_dist == float('inf'):
            min_dist = 5.0 # Max meaningful clearance cap
            
        if min_dist < self.robot.radius + 0.15: # Collision + Buffer
            return -float('inf')
            
        obstacle_score = min_dist
        
        # 3. Velocity
        velocity_score = v
        
        total = (self.to_goal_cost_gain * goal_score + 
                 self.obstacle_cost_gain * obstacle_score + 
                 self.speed_cost_gain * velocity_score)
        return total
