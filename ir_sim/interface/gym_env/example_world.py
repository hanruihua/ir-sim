import gymnasium as gym
from ir_sim2.interface.gym_env import GymEnvBase

class GymWorld(gym.Env):
    def __init__(self, world_name, render_mode='human', **kwargs) -> None:
        self.ir_env = GymEnvBase(world_name, **kwargs)

    def step(self, action):
        self.ir_env.step(action)

        observation = self.ir_env._get_observation()
        reward = self.ir_env._get_reward()
        info = self.ir_env._get_info()
        terminated = self.ir_env._get_terminated()
        truncated = self.ir_env._get_truncated()

        return observation, reward, terminated, truncated, info
    
    def reset(self, seed=None):
        super().reset(seed=seed)

        self.ir_env.reset()

    def render(self):
        self.ir_env.render()