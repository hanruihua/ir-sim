import numpy as np
from utils import get_transform, distance, wrap_to_pi

class Lidar2D:
    def __init__(self, robot, max_range=5.0, num_beams=36, noise_std=0.0):
        self.robot = robot
        self.max_range = max_range
        self.num_beams = num_beams
        self.noise_std = noise_std
        
        # Angles for each beam relative to robot
        self.angles = np.linspace(-np.pi, np.pi, num_beams, endpoint=False)
        self.last_scan = None
        
    def step(self, obstacles):
        """
        Simulate a scan.
        Returns: np.array of shape (N, 2) containing [range, angle] for valid hits.
        (For simplicity, we return points or ranges. Let's return ranges first)
        """
        # Robot Pose
        rx, ry, rtheta = self.robot.state
        
        ranges = np.full(self.num_beams, self.max_range)
        
        # Very simple ray casting against circles (Obstacles)
        # TODO: Optimize this for many obstacles if needed
        
        for i, angle in enumerate(self.angles):
            global_angle = wrap_to_pi(rtheta + angle)
            # Ray direction
            dx = np.cos(global_angle)
            dy = np.sin(global_angle)
            
            min_dist = self.max_range
            
            for obs in obstacles:
                if obs.shape == 'circle':
                    # Check intersection between Ray(rx, ry, dx, dy) and Circle(ox, oy, r)
                    ox, oy = obs.state
                    r = obs.radius
                    
                    fx = ox - rx
                    fy = oy - ry
                    
                    t_proj = fx * dx + fy * dy
                    dist_line_sq = (fx*fx + fy*fy) - t_proj*t_proj
                    
                    if dist_line_sq > r*r: continue
                    if t_proj < 0:
                        if np.sqrt(fx*fx + fy*fy) < r: min_dist = 0 # Inside
                        continue
                        
                    dt = np.sqrt(r*r - dist_line_sq)
                    t_hit = t_proj - dt
                    
                    if 0 < t_hit < min_dist:
                        min_dist = t_hit
                
                elif obs.shape == 'rectangle':
                    # Ray-Rectangle Intersection (4 Segments)
                    ox, oy = obs.state
                    w, h = obs.width, obs.height
                    theta = obs.theta
                    
                    # 1. Get Corners
                    # Local corners (unrotated, centered at 0)
                    corners_local = [
                        [-w/2, -h/2], [w/2, -h/2], [w/2, h/2], [-w/2, h/2] 
                    ]
                    # Rotate and Translate
                    c, s = np.cos(theta), np.sin(theta)
                    R = np.array([[c, -s], [s, c]])
                    corners_global = []
                    for cl in corners_local:
                        pg = np.dot(R, cl) + np.array([ox, oy])
                        corners_global.append(pg)
                        
                    # 2. Check 4 Segments
                    for k in range(4):
                        p1 = corners_global[k]
                        p2 = corners_global[(k+1)%4]
                        
                        # Ray: P = R + t*D
                        # Segment: S = P1 + u*(P2-P1)
                        # R + t*D = P1 + u*(P2-P1)
                        # t*D - u*(P2-P1) = P1 - R
                        # Solve for t, u
                        v1 = [dx, dy]
                        v2 = [p2[0]-p1[0], p2[1]-p1[1]] # Segment vector
                        v3 = [p1[0]-rx, p1[1]-ry] # Ray Origin to Seg Start
                        
                        # Cramers Rule / Cross Product
                        # Matrix [v1x  -v2x] [t] = [v3x]
                        #        [v1y  -v2y] [u]   [v3y]
                        det = v1[0]*(-v2[1]) - v1[1]*(-v2[0])
                        # det = - (v1x v2y - v1y v2x)
                        # det = - cross(v1, v2)
                        
                        if abs(det) < 1e-6: continue # Parallel
                        
                        # t = (v3x(-v2y) - v3y(-v2x)) / det
                        t_seg = (v3[0]*(-v2[1]) - v3[1]*(-v2[0])) / det
                        # u = (v1x v3y - v1y v3x) / det
                        u_seg = (v1[0]*v3[1] - v1[1]*v3[0]) / det
                        
                        if 0 <= u_seg <= 1 and t_seg > 0:
                            if t_seg < min_dist:
                                min_dist = t_seg
            
            # Add noise
            if self.noise_std > 0 and min_dist < self.max_range:
                min_dist += np.random.normal(0, self.noise_std)
                
            ranges[i] = min_dist
            
        self.last_scan = ranges
        return ranges

    def get_point_cloud(self):
        """
        Convert ranges to global (x,y) points
        """
        if self.last_scan is None:
            return np.empty((0, 2))
            
        points = []
        rx, ry, rtheta = self.robot.state
        
        for i, r in enumerate(self.last_scan):
            if r >= self.max_range: 
                continue # No hit
                
            angle = wrap_to_pi(rtheta + self.angles[i])
            px = rx + r * np.cos(angle)
            py = ry + r * np.sin(angle)
            points.append([px, py])
            
        return np.array(points)
