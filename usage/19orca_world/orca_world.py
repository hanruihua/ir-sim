import time

import pyrvo

import irsim

if __name__ == "__main__":
    orca = pyrvo.RVOSimulator()

    env = irsim.make(save_ani=False, display=True)

    orca.set_time_step(env.step_time)
    orca.set_agent_defaults(15.0, 10, 20.0, 10.0, 1.5, 2.0)

    for _i, robot in enumerate(env.robot_list):
        orca.add_agent(robot.state[:2, 0].tolist())

    while True:
        start_time = time.time()

        for i, robot in enumerate(env.robot_list):
            orca.set_agent_pref_velocity(
                i, robot.get_desired_omni_vel(normalized=True).flatten().tolist()
            )
            orca.set_agent_position(i, robot.state[:2, 0].tolist())

        orca.do_step()
        action_list = [
            orca.get_agent_velocity(i).to_tuple() for i in range(orca.get_num_agents())
        ]

        env.step(action_list)
        end_time = time.time()
        print(f"Time taken: {end_time - start_time} seconds")
        env.render()

        if env.done():
            break

    env.end()
