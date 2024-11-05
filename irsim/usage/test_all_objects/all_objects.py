import irsim

env = irsim.make()

for i in range(500):

    env.step()
    env.render(0.05)

env.end()
