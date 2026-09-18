import irsim

env = irsim.make("push_box_keyboard.yaml")

for box in env.obstacle_list:
    box.set_text(f"{box.mass:g} kg")

env.robot.set_text(f"{env.robot.mass:g} kg")

while True:
    env.step()
    env.render(0.02)

    if env.done():
        break

env.end()
