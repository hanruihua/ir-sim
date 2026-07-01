import irsim

# Fog of map over an obstacle (grid) map. Drive the robot with the keyboard to
# clear the fog and reveal the cave map (free space white, walls black) as you
# explore. The obstacle map (world.grid_map) and the fog overlay (world.fog_map)
# coexist: the fog simply hides the map until the lidar sees each area.
env = irsim.make("fog_grid_map.yaml")

for _ in range(10000):
    env.step()
    env.render(0.05)

    if env.done():
        break

env.end()
