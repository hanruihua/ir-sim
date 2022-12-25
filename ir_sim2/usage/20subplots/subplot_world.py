from ir_sim2.env import EnvBase

env = EnvBase('subplot_world.yaml', control_mode='keyboard', collision_mode='react', subplot_num=2)
# env = EnvBase('collision_mode_car.yaml', control_mode='keyboard', collision_mode='stop')
# subplot_num: only 0, 1, 2, 3  default: 1

for i in range(3000):
    env.step()
    env.ax2.plot(env.robot.state[0, 0], env.robot.state[1, 0], 'ro')
    env.render()
    
env.end()
