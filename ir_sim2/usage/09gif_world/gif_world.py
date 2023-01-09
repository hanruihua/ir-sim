from ir_sim2.env import EnvBase

env = EnvBase('gif_world.yaml', disable_all_plot=False, display=True, save_ani=True, save_fig=False, full=False, bbox_inches='tight', ani_dpi=300, fig_dpi=600)
# env = EnvBase('gif_world.yaml', save_ani=True, full=False, image_path='./test', ani_path=Path('./test2'), display=False)

for i in range(100):

    vel = env.cal_des_vel()
    env.step(vel)

    env.render(show_text=True)
    env.reset('single') 

# env.end(ani_name='gif_world', show_text=True, )
env.end(ani_name='gif_world_duration', show_text=True, show=False, ani_kwargs={'duration': 1, 'subrectangles': True})
# ani_args for GIF animation: https://imageio.readthedocs.io/en/v2.8.0/format_gif-pil.html#gif-pil

