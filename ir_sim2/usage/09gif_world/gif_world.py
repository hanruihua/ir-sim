from ir_sim2.env import EnvBase

env = EnvBase('gif_world.yaml', save_ani=True, full=True)

for i in range(1000):

    vel = env.cal_des_vel()
    env.step(vel)
    env.render(show_text=True, bbox_inches='tight')
    env.reset('single') 

env.end(ani_name='gif_world', show_text=True, show=False)
