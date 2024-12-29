import irsim

env = irsim.make(save_ani=False)

for i in range(3000):

    env.step()
    # env.robot.set_state([0.5*i, 0.5*i, i])       
    
    for obs in env.obstacle_list:
        if obs.fov_detect_object(env.robot):
            print(f'The robot is in the FOV of the {obs.name}. The parameters of this obstacle are: state [x, y, theta]: {obs.state.flatten()}, velocity [linear, angular]: {obs.velocity.flatten()}, fov in radian: {obs.fov}.')

    env.render()

    if env.done():
        break

env.end()
