import gym

from baselines import deepq
from p2os_test.src.seventh_edition_gpw import set_gpw_num
import time


def main():
    set_gpw_num(3)  # 设置环境号
    env = gym.make("SeventhRobGpw-v0")
    act = deepq.load("asyn_rob_model.pkl")
    time.sleep(2)

    while True:
        obs, done = env.reset(), False
        episode_rew = 0
        while not done:
            # env.render()
            obs, rew, done, _ = env.step(act(obs[None])[0])
            episode_rew += rew
        print("Episode reward", episode_rew)


if __name__ == '__main__':
    main()
