
class world:
    def __init__(self, height=10, width=10, step_time=0.01, sample_time=0.1, offset=[0, 0], landmarks=[]) -> None:

        self.height = height
        self.width = width
        self.step_time = step_time
        self.sample_time = sample_time
        self.offset = offset
        self.count = 0
        self.sampling = True
        self.x_range = [self.offset[0], self.offset[0] + self.width]
        self.y_range = [self.offset[1], self.offset[1] + self.width]

    def step(self):
        self.count += 1
        self.sampling = (self.count % (self.sample_time / self.step_time) == 0)
        
    