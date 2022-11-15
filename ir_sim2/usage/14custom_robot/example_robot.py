from ir_sim2.world import RobotBase

class RobotCustom(RobotBase):
    def __init__(self, id, state, vel, goal=..., step_time=0.1, vel_min=..., vel_max=..., acce=..., **kwargs):
        super().__init__(id, state, vel, goal, step_time, vel_min, vel_max, acce, **kwargs)
    
    

    # def 