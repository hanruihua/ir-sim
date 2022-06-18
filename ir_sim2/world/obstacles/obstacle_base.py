
class ObstacleBase:
    def __init__(self, id, type='circle', ):
        # self.shape
        self.A, self.b = self.gen_inequal()
    
    def collision_check(self):
        pass
    
    def gen_inequal(self):
        # Calculate the matrix A and b for the Generalized inequality: G @ point <_k g, 
        # self.G, self.g = self.gen_inequal()
        raise NotImplementedError
    
    def plot(self):
        pass

    def plot_clear(self):
        pass

    # def 