import irsim
from irsim.global_param.path_param import path_manager

path_manager.ani_path = path_manager.root_path + '/usage/16fov_world/test_123'
path_manager.ani_buffer_path = path_manager.root_path + '/usage/16fov_world/test_123_buffer'
full_path = path_manager.root_path + '/usage/16fov_world/fov_world.yaml'

env = irsim.make(full_path, save_ani=True)

for i in range(10):

    env.step()
    # env.robot.set_state([x, y, theta])       
    # env.robot.set_goal([10, 30, 0])

    for obs in env.obstacle_list:
        if obs.fov_detect_object(env.robot):
            print(f'The robot is in the FOV of the {obs.name}. The parameters of this obstacle are: state [x, y, theta]: {obs.state.flatten()}, velocity [linear, angular]: {obs.velocity.flatten()}, fov in radian: {obs.fov}.')

    env.render()

    if env.done():
        break

env.end()
