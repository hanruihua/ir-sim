import irsim

'''
- Refer to the irsim/lib/behavior_methods.py file for the custom behavior design.
- custom_behavior_methods.py is the designated module name. It should be placed in the same directory as the current implementation script.
- The behavior names defined in custom_behavior_methods.py (e.g., dash_custom) must match the behavior names specified in the YAML file.
'''

env = irsim.make()
env.load_behavior("custom_behavior_methods")

for i in range(1000):

    env.step()
    env.render(0.01)
    
    if env.done():
        break

env.end(5)
