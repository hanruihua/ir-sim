import irsim

env = irsim.make("push_box_world.yaml")
boxes = env.obstacle_list[:3]

for box in boxes:
    box.set_text(f"m = {box.mass:g} kg")

for _ in range(200):
    env.step()
    env.render(0.02)

    if env.done():
        break

for robot, box in zip(env.robot_list, boxes, strict=True):
    print(
        f"{box.name} (mass {box.mass:g} kg) moved {box.state[0, 0] - 3:.2f} m, "
        f"pushed by {robot.name} at {robot.velocity[0, 0]:.2f} m/s"
    )

env.end(3)
