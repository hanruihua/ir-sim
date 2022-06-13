from ir_sim2.env import EnvBase


env = EnvBase(world_name = 'robot_world.yaml')

for i in range(300):

    des_vel = env.robot.cal_des_vel()

    env.robot_step(des_vel)
    env.render()

