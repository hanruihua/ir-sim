from ir_sim2.env import EnvBase

world_name = 'car_world.yaml'
env = EnvBase(world_name = world_name, plot=True, init_mode=0, robot_mode='diff')

for i in range(300):
    des_vel = env.car.cal_des_vel()

    env.robot_step(des_vel)
    env.render()

env.show()
