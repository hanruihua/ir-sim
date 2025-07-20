from irsim.world.world import World


class World3D(World):

    def __init__(self, name, depth: float = 10.0, offset=[0, 0, 0], **kwargs):
        super().__init__(name=name, **kwargs)

        self.depth = depth

        self.offset = offset if len(offset) == 3 else [offset[0], offset[1], 0]

        self.z_range = [self.offset[2], self.offset[2] + self.depth]
