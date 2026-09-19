import irsim

env = irsim.make("push_box_keyboard.yaml")
robot = env.robot
dt = env.step_time
touching = False


def label(obj):
    """Mass, plus the net contact force on the object when something touches it."""
    fx, fy = obj.contact_force.ravel()
    force = (fx**2 + fy**2) ** 0.5
    obj.set_text(f"{obj.mass:g} kg" + (f"\n{force:.1f} N" if force > 0 else ""))


while True:
    env.step()
    env.render(0.02)

    for obj in env.objects:
        label(obj)

    # the robot's contact report, printed when a contact starts or ends
    if robot.contact_time == dt:
        touching = True
        for contact in robot.contacts:
            other = contact.b if contact.a is robot else contact.a
            px, py = contact.point
            print(
                f"t={env.time:.1f}s touching {other.name} ({other.mass:g} kg) at "
                f"({px:.2f}, {py:.2f}), force {contact.force:.1f} N"
            )
    elif touching and robot.air_time == dt:
        touching = False
        print(f"t={env.time:.1f}s free again")

    if env.done():
        break

env.end()
