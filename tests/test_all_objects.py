import irsim

def test_all_objects():

    env1 = irsim.make("test_collision_avoidance.yaml", save_ani=False, full=False, display=False)

    for i in range(10):
        env1.step()
        env1.render(0.01)
        
    env1.end()
    
    env2 = irsim.make('test_all_objects.yaml', display=False)

    for i in range(20):
        env2.step()
        env2.render(0.01)
    env2.end()

    env3 = irsim.make('test_keyboard_control.yaml', save_ani=False, display=False)

    for i in range(30):
        env3.step()
        env3.render(0.01)
    env3.end()

if __name__ == "__main__":
    test_all_objects()
