from pathlib import Path
import sys

import irsim
from cbf_qp import CBFQPController

scene_name = sys.argv[1] if len(sys.argv) > 1 else "cbf_world.yaml"
scene_path = Path(scene_name)
if not scene_path.is_absolute():
    workspace_scene_path = (Path.cwd() / scene_path).resolve()
    local_scene_path = (Path(__file__).parent / scene_path).resolve()
    scene_path = workspace_scene_path if workspace_scene_path.exists() else local_scene_path

env = irsim.make(str(scene_path), save_ani=True, display=True)
env.path_param.ani_path = str(scene_path.parent)
env.path_param.ani_buffer_path = str(scene_path.parent / f"{scene_path.stem}_buffer")
controller = CBFQPController(
    robot_type=env.robot.kinematics,
    safety_margin=0.15,
    cbf_alpha=2.0,
    goal_gain=0.8,
)

for i in range(400):
    action = controller.get_action(env.robot, env.obstacle_list)
    env.step(action)
    env.render(0.01)
    # if i>200:
    #     import pdb; pdb.set_trace()
    if env.done():
        break

env.end(ani_name=scene_path.stem, suffix=".gif")
