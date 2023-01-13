from ir_sim.env import EnvBase

env = EnvBase('multi_robot_car.yaml')
# env = EnvBase('multi_robot.yaml')

for i in range(3000):

    vel = env.cal_des_vel()
    env.step(vel)
    env.render(0.05)

    if env.done():
        break

env.end()
