from c3bf_qp import CollisionConeCBFController

import irsim

env = irsim.make('cbf_world.yaml', save_ani=False, display=True)
controller = CollisionConeCBFController(
    robot_type=env.robot.kinematics,
    safety_margin=0.15,
    goal_gain=0.8,
)

for _ in range(400):
    action = controller.get_action(env.robot, env.obstacle_list)
    env.step(action)
    env.render(0.01)

    if env.done():
        break

env.end()
