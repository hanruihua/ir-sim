from ir_sim.env import EnvBase

env = EnvBase('line_obstacle.yaml', control_mode='keyboard', collision_mode='react')

for i in range(3000):

    env.step()
    env.render(show_text=True)
        
env.end(show_text=True)
