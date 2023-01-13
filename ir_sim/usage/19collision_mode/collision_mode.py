from ir_sim.env import EnvBase

env = EnvBase('collision_mode.yaml', control_mode='keyboard', collision_mode='react')
# env = EnvBase('collision_mode_car.yaml', control_mode='keyboard', collision_mode='stop')

for i in range(3000):

    env.step()
    env.render(show_text=True)
     
env.end(show_text=False)
