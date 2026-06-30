import irsim

# Fog of map cleared by the robot's field of view (FOV) rather than a lidar.
# The robot has no sensors -- its fov / fov_radius sector reveals the fog as it
# wanders, so the FOV wedge (show_fov) sweeps open the explored area.
env = irsim.make("fog_fov_world.yaml")

for _ in range(1000):
    env.step()
    env.render(0.05)

    if env.done():
        break

env.end()
