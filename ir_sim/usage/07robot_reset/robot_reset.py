from ir_sim.env import EnvBase

env = EnvBase('robot_reset.yaml')

for i in range(3000):

    vel = env.cal_des_vel()
    env.step(vel)
    env.render(0.05)

    env.reset('single')  # 'all'; 'any'; 'single'

env.end()
