from ir_sim.env import EnvBase

env = EnvBase('subplot_world.yaml', control_mode='keyboard', collision_mode='react', subplot=True)
# env = EnvBase('collision_mode_car.yaml', control_mode='keyboard', collision_mode='stop')
 
for i in range(100):
    env.step()
    env.sub_ax_list[0].plot(env.robot.state[0, 0], env.robot.state[1, 0], 'ro')
    env.render()

env.end()


