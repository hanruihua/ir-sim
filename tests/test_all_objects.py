import irsim

def test_all_objects():

    env = irsim.make('test_all_objects.yaml', display=False)

    for i in range(100):

        env.step()
        env.render(0.05)

    env.end()

if __name__ == "__main__":
    test_all_objects()
