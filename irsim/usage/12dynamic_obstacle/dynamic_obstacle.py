import irsim

env = irsim.make()

for i in range(3000):

    env.step()
    env.render(0.05)

    if env.done():
        break


env.end()
