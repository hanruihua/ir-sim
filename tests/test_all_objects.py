import irsim
from irsim.global_param import world_param, env_param

def test_collision_avoidance():

    env2 = irsim.make("test_collision_avoidance.yaml", save_ani=False, full=False, display=False)

    for i in range(100):

        env2.step()
        env2.render(0.01)
    
    print(env_param.objects)

    env2.end()


def test_all_objects():

    env = irsim.make('test_all_objects.yaml', display=False)

    for i in range(100):

        env.step()
        env.render(0.05)
        
    print(env_param.objects)

    env.end()




# if __name__ == "__main__":
#     test_all_objects()
