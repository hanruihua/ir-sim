import irsim

env = irsim.make()

env.random_obstacle_position(
    [1, 2, -3.14], [13, 11, 3.14], [5, 6, 7, 8, 9, 10, 11, 12], True
)
env.reset()

for _i in range(1000):
    env.step()
    env.render()

    if env.done():
        env.random_obstacle_position(
            [1, 2, -3.14], [13, 11, 3.14], [5, 6, 7, 8, 9, 10, 11, 12], True
        )
        env.reset()

env.end()
