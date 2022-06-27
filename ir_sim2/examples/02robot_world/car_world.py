from ir_sim2.env import EnvBase

env = EnvBase(world_name = 'car_world.yaml')

for i in range(300):
    des_vel = env.cal_des_vel()
    env.step(des_vel)
    env.render(0.05, show_trail=False, show_traj=True)

    if env.done():
        break

env.show(show_trail=True, show_traj=True)