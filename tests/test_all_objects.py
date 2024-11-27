import irsim

def test_all_objects():

    env = irsim.make("test_collision_avoidance.yaml", save_ani=False, full=False, display=False)

    for i in range(10):
        env.step()
        env.render(0.01)
        
    env.end()
    
    env = irsim.make('test_all_objects.yaml', display=False)

    for i in range(10):
        env.step()
        env.render(0.01)
    env.end()

    env = irsim.make('test_keyboard_control.yaml', save_ani=False, display=False)

    for i in range(10):
        env.step()
        env.render(0.01)
    env.end()


if __name__ == "__main__":
    test_all_objects()
