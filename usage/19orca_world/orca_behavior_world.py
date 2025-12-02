import time

import irsim

if __name__ == "__main__":
    env = irsim.make(save_ani=False, display=True)

    while True:
        start_time = time.time()
        env.step()

        end_time = time.time()
        print(f"Time taken: {end_time - start_time} seconds")

        env.render()

        if env.done():
            break

    env.end()
