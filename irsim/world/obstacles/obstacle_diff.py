from irsim.world import ObjectBase

class ObstacleDiff(ObjectBase):
    def __init__(self, color="k", **kwargs):
        super(ObstacleDiff, self).__init__(color=color, role="obstacle", **kwargs
        )
