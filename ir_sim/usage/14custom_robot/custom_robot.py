from example_robot import RobotCustom
from ir_sim.env import EnvBase

env = EnvBase('custom_robot.yaml', control_mode='keyboard', custom_robot=RobotCustom)

for i in range(3000):
    env.step()
    env.render(show_traj=True, show_text=True)
    env.reset('any')  # 'all'; 'any'

env.end(show_traj=True, show_text=True)
