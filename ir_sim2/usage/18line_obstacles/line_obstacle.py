from ir_sim2.env import EnvBase

env = EnvBase('collision_mode.yaml', control_mode='keyboard', collision_mode='stop')

for i in range(3000):

    env.step()
    env.render(show_text=True)
    
    env.reset('any')  # 'all'; 'any'
    
env.end(show_text=True)
