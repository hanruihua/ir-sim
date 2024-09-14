from irsim.lib.kinematics import (
    differential_kinematics,
    ackermann_kinematics,
    omni_kinematics,
)

kinematics_factory = {
    "diff": differential_kinematics,
    "acker": ackermann_kinematics,
    "omni": omni_kinematics,
}
