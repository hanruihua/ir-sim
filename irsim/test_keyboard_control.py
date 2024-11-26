import irsim

def test_keyboard_control(): 

    env = irsim.make('test_keyboard_control.yaml', save_ani=False, display=False)

    for i in range(100):

        env.step()
        env.render(0.05, show_goal=False, show_trajectory=True)
        
    env.end()


if __name__ == "__main__":
    test_keyboard_control()