import time
import numpy as np
import matplotlib.pyplot as plt
from world import World
from entities import Robot, Obstacle
from sensor import Lidar2D
from mapping import OccupancyGrid
from navigation import AStarPlanner, DWAController
from utils import distance, wrap_to_pi

def main():
    # 1. Initialize World from YAML
    yaml_file = 'robot_world.yaml'
    print(f"Loading config from {yaml_file}...")
    world, config_data = World.from_yaml(yaml_file)
    
    # Instantiate Robots from Config
    if 'robots' in config_data:
        for r_conf in config_data['robots']:
            state = r_conf.get('state', [0,0,0])
            goal = r_conf.get('goal', [0,0])
            kinematics = r_conf.get('kinematics', {}).get('name', 'diff')
            radius = r_conf.get('shape', {}).get('radius', 0.2)
            color = r_conf.get('color', 'b')
            
            bot = Robot(x=state[0], y=state[1], theta=state[2], 
                        radius=radius, goal=goal, kinematics=kinematics)
            bot.color = color
            world.add_robot(bot)
    
    if not world.robots:
        print("No robots found in YAML!")
        return

    # 2. Initialize Perception & Planning for ALL robots
    robot_modules = [] # List of dicts: {robot, lidar, grid, planner, dwa, path}
    
    width, height = world.width, world.height
    
    for i, bot in enumerate(world.robots):
        print(f"Initializing Robot {i} (kinematics={bot.kinematics})...")
        
        # Sensor & Map
        lidar = Lidar2D(bot, max_range=5.0)
        bot.set_lidar(lidar)
        
        grid = OccupancyGrid(width=width, height=height, resolution=0.2)
        planner = AStarPlanner(grid)
        dwa = DWAController(bot)
        
        # Store modules
        robot_modules.append({
            'robot': bot,
            'lidar': lidar,
            'grid': grid,
            'planner': planner,
            'dwa': dwa,
            'path': [],
            'goal': bot.goal,
            'color': 'r' if i==1 else 'r' # path color
        })

    # Metrics Variables (Global)
    start_time = time.time()
    
    try:
        print("Starting Multi-Robot Simulation... Press Ctrl+C to stop.")
        step_count = 0
        
        while True:
            # --- Loop over all robots ---
            for mod in robot_modules:
                bot = mod['robot']
                lidar = mod['lidar']
                grid = mod['grid']
                planner = mod['planner']
                dwa = mod['dwa']
                goal = mod['goal']
                
                # 1. Perception
                # (For multi-robot, obstacles should ideally include other robots? 
                #  Current Lidar only checks static obstacles. improving this is complex (dynamic obstacle list).
                #  Let's stick to static obstacles for now for map building)
                ranges = lidar.step(world.obstacles) 
                point_cloud = lidar.get_point_cloud()
                grid.update(bot.state, point_cloud)
                
                # 2. Global Planning
                if step_count % 10 == 0:
                    mod['path'] = planner.plan(bot.state[:2], goal)
                
                path = mod['path']
                
                # 3. Control
                # Strict Path Dependency
                if (not path or len(path) <= 1) and step_count > 10:
                    v, w = 0.0, 0.5 # Scan
                else:
                    local_goal = goal
                    if path and len(path) > 1:
                        idx = min(len(path)-1, 2)
                        local_goal = path[idx]
                    v, w = dwa.plan(point_cloud, local_goal)
                
                bot.set_velocity(v, w)
            
            # --- 4. Simulation Step (Physics) ---
            world.step()
            
            # --- 5. Render (Every 5 steps) ---
            if step_count % 5 == 0:
                world.ax.cla()
                world.ax.set_xlim(0, width)
                world.ax.set_ylim(0, height)
                world.ax.set_aspect('equal')
                world.ax.grid(True)
                
                # Show Static Obstacles
                for obs in world.obstacles:
                    c = plt.Circle(obs.state, obs.radius, color='gray', alpha=0.3)
                    world.ax.add_patch(c)

                # Render Each Robot's View
                for i, mod in enumerate(robot_modules):
                    bot = mod['robot']
                    grid = mod['grid']
                    path = mod['path']
                    
                    # Show Map (Only show Robot 0's map to avoid clutter, or average?)
                    # Let's show Robot 0's map
                    if i == 0:
                        grid.render(world.ax)
                    
                    # Show Path
                    if path:
                        px = [p[0] for p in path]
                        py = [p[1] for p in path]
                        world.ax.plot(px, py, f'{mod["color"]}-', linewidth=1)
                    
                    # Show Robot
                    bot.render(world.ax)
                    
                    # Show Lidar Rays (Optional, can be messy for multiple)
                    # Let's show rays for all
                    rx, ry, rtheta = bot.state
                    ranges = mod['lidar'].last_scan
                    if ranges is not None:
                        lidar_points = []
                        for j, r in enumerate(ranges):
                            angle = wrap_to_pi(rtheta + mod['lidar'].angles[j])
                            px = rx + r * np.cos(angle)
                            py = ry + r * np.sin(angle)
                            lidar_points.append([px, py])
                        
                        from matplotlib.collections import LineCollection
                        ray_lines = [([rx, ry], pt) for pt in lidar_points]
                        lc = LineCollection(ray_lines, colors='r', linewidths=0.5, alpha=0.1)
                        world.ax.add_collection(lc)
                    
                    # Show Goal
                    if mod['goal'] is not None:
                       world.ax.plot(mod['goal'][0], mod['goal'][1], 'rx', markersize=10)

                world.ax.set_title(f"Multi-Robot Simulation | Step: {step_count}")
                plt.draw()
                plt.pause(0.001)
            
            step_count += 1
            
            # Check Goal (Any robot reached?)
            # for mod in robot_modules: ...
                
    except KeyboardInterrupt:
        print("\nSimulation Stopped by User.")
    except Exception as e:
        print(f"Simulation Error: {e}")
        import traceback
        traceback.print_exc()

    print("\nSimulation Ended.")
    try:
        plt.show(block=True)
    except: pass

if __name__ == "__main__":
    main()
