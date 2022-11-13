from ir_sim2.env import EnvBase

# env = EnvBase('robot_world.yaml')
env = EnvBase('robot_omni_world.yaml')
# env = EnvBase('car_world.yaml')

for i in range(3000):

    des_vel = env.cal_des_vel()
    env.step(des_vel)
    env.render(0.05)

    if env.done():
        break

env.end()
    
