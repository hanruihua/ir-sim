
class robot_base:
    def __init__(self, id, init_state, init_vel, vel_limit, goal_state, goal_threshold, shape, step_time=0.1, **kwargs):
        self.id = int(id)
        self.step_time = step_time

        if isinstance(init_state, list): 
            init_state = np.array(init_state, ndmin=2).T

    def move(self, vel, stop=True, **kwargs):
        """ vel: velocity to control the robot
            stop: the robot will stop when arriving at the goal
        """
        raise NotImplementedError
    
    @staticmethod
    def list_to_np(list):
        pass


    
