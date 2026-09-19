import irsim

env = irsim.make("push_box_world.yaml")
boxes = env.obstacle_list[:3]
dt = env.step_time
touching = set()  # robots currently reporting a contact


def label(box):
    """Mass and the contact force the solver reports on the box, in newtons."""
    fx, fy = box.contact_force.ravel()
    box.set_text(f"m = {box.mass:g} kg\nF = {(fx**2 + fy**2) ** 0.5:.1f} N")


for _ in range(200):
    env.step()
    env.render(0.02)

    for box in boxes:
        label(box)

    for robot in env.robot_list:
        # what a contact sensor would report: partner, point, normal, force
        if robot.contact_time == dt:  # this step started a contact
            touching.add(robot)
            for contact in robot.contacts:
                other = contact.b if contact.a is robot else contact.a
                px, py = contact.point
                print(
                    f"t={env.time:.1f}s {robot.name} touches {other.name} "
                    f"({other.mass:g} kg) at ({px:.2f}, {py:.2f}), "
                    f"force {contact.force:.1f} N along {contact.normal.round(2)}"
                )
        elif robot in touching and robot.air_time == dt:  # this step ended one
            touching.discard(robot)
            print(f"t={env.time:.1f}s {robot.name} lost contact")

    if env.done():
        break

for robot, box in zip(env.robot_list, boxes, strict=True):
    fx, fy = robot.contact_force.ravel()
    print(
        f"{box.name} ({box.mass:g} kg) moved {box.state[0, 0] - 3:.2f} m; "
        f"{robot.name} drives at {robot.velocity[0, 0]:.2f} m/s, feels "
        f"{(fx**2 + fy**2) ** 0.5:.1f} N and has been in contact for "
        f"{robot.contact_time:.1f} s"
    )

env.end(3)
