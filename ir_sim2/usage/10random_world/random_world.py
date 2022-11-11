from ir_sim2.env import EnvBase

env = EnvBase('random_world.yaml')

for i in range(3000):

    vel = env.cal_des_vel()
    env.step(vel)
    env.render()

    env.reset('single')
    
env.end()
