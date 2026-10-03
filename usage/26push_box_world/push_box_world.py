import irsim

env = irsim.make("push_box_world.yaml")
boxes = env.obstacle_list[:3]


def label(box):
    """Mass and the contact force the solver reports on the box, in newtons."""
    fx, fy = box.contact.force.ravel()
    box.set_text(f"m = {box.mass:g} kg\nF = {(fx**2 + fy**2) ** 0.5:.1f} N")


for _ in range(200):
    env.step()
    env.render(0.02)

    for box in boxes:
        label(box)

    for robot in env.robot_list:
        # each robot's contact report: partner, point, force and normal
        if robot.contact.started:
            for contact in robot.contact.reports:
                print(f"t={env.time:.1f}s {robot.name} touches {contact}")
        elif robot.contact.ended:
            print(f"t={env.time:.1f}s {robot.name} lost contact")

    if env.done():
        break

for robot, box in zip(env.robot_list, boxes, strict=True):
    fx, fy = robot.contact.force.ravel()
    print(
        f"{box.name} ({box.mass:g} kg) moved {box.state[0, 0] - 3:.2f} m; "
        f"{robot.name} drives at {robot.velocity[0, 0]:.2f} m/s, feels "
        f"{(fx**2 + fy**2) ** 0.5:.1f} N and has been in contact for "
        f"{robot.contact.contact_time:.1f} s"
    )

env.end(3)
