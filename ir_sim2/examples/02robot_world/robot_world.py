from ir_sim2.env import EnvBase

robot_args = {'number': 12}

env = EnvBase(world_name = 'robot_world.yaml', robot_args=robot_args)

for i in range(300):

    des_vel = env.cal_des_vel()
    env.step(des_vel)
    env.render(0.05)

    if env.done():
        break

env.show()
    
