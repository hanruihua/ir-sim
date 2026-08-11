import irsim

env = irsim.make("msg_world.yaml")

# A message from another simulator or a ROS bridge can initialize IR-SIM.
external_odom = irsim.msg.Odometry()
external_odom.pose.pose.position.x = 1.5
external_odom.pose.pose.position.y = 1.0
external_odom.pose.pose.orientation = irsim.msg.Quaternion.from_yaw(0.2)
external_odom.twist.twist.linear.x = 0.3
env.receive_msg(external_odom, object_name="message_robot")

step = 0
while not env.done() and step < 500:
    env.step()

    msg = env.get_msg()
    robot = msg.robots[0]
    odom = robot.odom
    assert robot.scan is not None
    scan = robot.scan
    payload = msg.to_dict()

    print(
        f"seq={msg.header.seq:03d} time={msg.header.stamp:5.2f}s "
        f"robot={robot.name} x={odom.pose.pose.position.x:5.2f} "
        f"y={odom.pose.pose.position.y:5.2f} "
        f"closest_range={scan.ranges.min():5.2f}m "
        f"payload_objects={len(payload['objects'])}"
    )

    env.render(0.05)
    step += 1

env.end(3)
