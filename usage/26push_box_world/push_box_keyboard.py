import irsim

env = irsim.make("push_box_keyboard.yaml")
robot = env.robot


def label(obj):
    """Mass, plus the net contact force on the object when something touches it."""
    fx, fy = obj.contact.force.ravel()
    force = (fx**2 + fy**2) ** 0.5
    obj.set_text(f"{obj.mass:g} kg" + (f"\n{force:.1f} N" if force > 0 else ""))


while True:
    env.step()
    env.render(0.02)

    for obj in env.objects:
        label(obj)

    # the robot's contact report, printed when a contact starts or ends
    if robot.contact.started:
        for contact_report in robot.contact.reports:
            print(f"t={env.time:.1f}s touching {contact_report}")
    elif robot.contact.ended:
        print(f"t={env.time:.1f}s free again")

    if env.done():
        break

env.end()
