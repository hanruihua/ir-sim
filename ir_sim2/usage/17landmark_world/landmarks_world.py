from ir_sim2.env import EnvBase

env = EnvBase('landmarks_world.yaml', control_mode='keyboard')

for i in range(3000):

    env.step()
    env.render(show_text=True)
    env.reset('any')  # 'all'; 'any'

    print(env.get_landmarks())  # get the landmarks information by lidar area
    # print(env.get_obstacles())  # get the landmarks information by lidar area
    
env.end(show_text=True)
