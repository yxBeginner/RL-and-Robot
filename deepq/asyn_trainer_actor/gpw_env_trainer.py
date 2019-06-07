import gym
import numpy as np
from gym.spaces import Discrete


class GpwTrainer(gym.Env):
    def __init__(self):
        # self.observation_space = gym.spaces.Box(0, 100, (12, ), dtype=np.dtype(np.int32))  # 整数版

        self.observation_space = gym.spaces.Box(-10.0, 10.0, (17, ), dtype=np.dtype(np.float32))
        self.action_space = Discrete(4)  # 取值0,1,2,3,4,5　，　动作是否应该加上组合？

    def reset(self):
        pass

    def step(self, action):
        pass

    def reward(self):
        pass

    def render(self, mode='human'):
        pass
