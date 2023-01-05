from ir_sim2.env import EnvBase

class GymEnvBase(EnvBase):
    def __init__(self, world_name=None, **kwargs) -> None:
        super().__init__(world_name, **kwargs)

    def _get_observation(self):
        raise NotImplementedError("The parent class _get_observation method not implemented")

    def _get_info(self):
        raise NotImplementedError("The parent class _get_info method not implemented")

    def _get_reward(self):
        raise NotImplementedError("The parent class _get_reward method not implemented")

    def _get_terminated(self):
        raise NotImplementedError("The parent class _get_terminated method not implemented")

    def _get_truncated(self):
        raise NotImplementedError("The parent class _get_truncated method not implemented")

    

    