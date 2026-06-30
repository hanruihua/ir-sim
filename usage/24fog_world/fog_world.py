import irsim

# A robot wanders behind a fog of map that its lidar clears along the line of
# sight, revealing the map (free space white, obstacles black) as it explores.
env = irsim.make("fog_world.yaml")

for _ in range(10000):
    env.step()
    env.render(0.05)

    if env.done():
        break

env.end()
