from ir_sim2.env import EnvBase

env = EnvBase('subplot_world.yaml', control_mode='keyboard', collision_mode='react')
# env = EnvBase('collision_mode_car.yaml', control_mode='keyboard', collision_mode='stop')

for i in range(3000):
    env.step()
    env.render()
     
env.end()
