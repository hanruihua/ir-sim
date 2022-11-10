from ir_sim2.env import EnvBase


env = EnvBase('robot_world.yaml')

for i in range(300):

    des_vel = env.cal_des_vel()
    env.step(des_vel)
    env.render(0.05)

    if env.done():
        break

env.show()
    
