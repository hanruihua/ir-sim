from ir_sim.lib.kinematics import differential_wheel_kinematics, ackermann_kinematics

kinematics_factory = {'diff': differential_wheel_kinematics, 'acker': ackermann_kinematics}