import numpy as np
import matplotlib.pyplot as plt

class OccupancyGrid:
    def __init__(self, width=10, height=10, resolution=0.1):
        self.width = width
        self.height = height
        self.resolution = resolution
        
        self.nx = int(width / resolution)
        self.ny = int(height / resolution)
        
        # Log-Odds Map: 0 = Unknown (p=0.5)
        self.grid_log_odds = np.zeros((self.nx, self.ny))
        
        # Parameters (Probabilities converted to log-odds)
        # p_occ = 0.9 => log(0.9/0.1) approx 2.2
        # p_free = 0.4 => log(0.4/0.6) approx -0.4
        
        self.lo_occ = 2.0  # Increment for hit
        self.lo_free = -0.5 # Decrement for miss
        self.lo_max = 20.0
        self.lo_min = -20.0
        
    def to_grid(self, x, y):
        """
        Convert physical (x, y) to grid indices (ix, iy)
        """
        ix = int(x / self.resolution)
        iy = int(y / self.resolution)
        
        if 0 <= ix < self.nx and 0 <= iy < self.ny:
            return ix, iy
        return None
        
    def update(self, robot_state, point_cloud):
        """
        Update grid using Bresenham ray tracing from robot to each point
        """
        rx, ry, _ = robot_state
        start_node = self.to_grid(rx, ry)
        
        if not start_node:
            return # Robot out of bounds?
            
        for pt in point_cloud:
            end_node = self.to_grid(pt[0], pt[1])
            if not end_node:
                continue
                
            # Ray trace
            cells = self.bresenham(start_node, end_node)
            
            # Update free cells (all except last)
            for cell in cells[:-1]:
                self.grid_log_odds[cell] += self.lo_free
                
            # Update occupied cell (last one)
            self.grid_log_odds[cells[-1]] += self.lo_occ
            
        # Clamp values
        np.clip(self.grid_log_odds, self.lo_min, self.lo_max, out=self.grid_log_odds)

    def bresenham(self, start, end):
        """
        Bresenham's Line Algorithm implementation
        Returns list of (x, y) tuples
        """
        x1, y1 = start
        x2, y2 = end
        
        cells = []
        
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy
        
        while True:
            cells.append((x1, y1))
            if x1 == x2 and y1 == y2:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy
        return cells

    def render(self, ax):
        # Convert log odds to probability
        # p = 1 / (1 + exp(-l))
        probs = 1.0 / (1.0 + np.exp(-self.grid_log_odds))
        
        # Transpose for correct plotting (x=col, y=row)
        ax.imshow(probs.T, origin='lower', extent=[0, self.width, 0, self.height], 
                  cmap='Greens', alpha=0.5, vmin=0, vmax=1)
