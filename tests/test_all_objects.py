import irsim

def test_all_objects():

    env = irsim.make('test_all_objects.yaml', display=False)

    for i in range(500):

        env.step()
        env.render(0.05)

    env.end()
