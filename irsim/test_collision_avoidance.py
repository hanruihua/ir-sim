import irsim


def test_collision_avoidance():

    env = irsim.make("test_collision_avoidance.yaml", save_ani=False, full=False, display=False)

    for i in range(100):

        env.step()
        env.render(0.01)

    env.end()


# if __name__ == "__main__":
#     test_collision_avoidance()