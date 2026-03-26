from cbf_qp import CBFQPController

import irsim

env = irsim.make('cbf_world_dynamic_diff.yaml', save_ani=False, display=True)

controller = CBFQPController(
    robot_type=env.robot.kinematics,
    safety_margin=0.15,
    cbf_alpha=2.0,
    goal_gain=0.8,
)

for _i in range(400):
    action = controller.get_action(env.robot, env.obstacle_list)
    env.step(action)
    env.render(0.01)

    if env.done():
        break

env.end()
