import numpy as np
from baselines import logger
import time


def actor_inter(env, ac_num, actor_deque, action_pipes, print_freq):

    # capture the shape outside the closure so that the env object is not serialized
    # by cloudpickle when serializing make_obs_ph

    # Initialize the parameters and copy them to the target network.
    # U.initialize()

    time.sleep(3)  # ros 节点注册等过程会占用一定时间,等待一下
    episode_rewards = [0.0]
    obs = env.reset()
    reset = True
    end = False
    t = 0

    while not end:
        if reset is True:
            mem = (ac_num, None, None, obs, None, None)  # 此时的obs是新一轮的起始observation
            actor_deque.put(mem)
        # print(str(ac_num)+" is in " + str(t)+"before")
        action = action_pipes.recv()
        # print(str(ac_num) + " is in " + str(t)+"end")
        if action is 100:  # 训练结束,一个不存在的动作,指代训练结束
            break
        # print(np.array(obs))
        # print(act(np.array(obs)[None], update_eps=update_eps, **kwargs))
        # print("action: "+str(action))
        reset = False
        new_obs, rew, done, _ = env.step(action)
        # Store transition in the replay buffer.
        # 这里直接将observation放入了buffer,DQN论文中则是将序列作为状态,也许是在atari_wrappers中已经做好了相关转换
        # print(obs)
        mem = (ac_num, obs, action, new_obs, rew, done)
        actor_deque.put(mem)
        # print(rew)
        obs = new_obs
        # 将即时回报加入回报序列
        episode_rewards[-1] += rew
        if done:
            # print(str(ac_num) + " is done")
            obs = env.reset()
            episode_rewards.append(0.0)
            reset = True
        t += 1
        # 平均100次情景的回报（）, 注意episode是轮数,不是步数
        # mean_100ep_reward = round(np.mean(episode_rewards[-101:-1]), 1)
        mean_20ep_reward = round(float(np.mean(episode_rewards[-21:-1])), 4)
        num_episodes = len(episode_rewards)
        # 下面是关于输出训练信息,以及保存网络参数的部分
        if done and print_freq is not None and len(episode_rewards) % print_freq == 0:
            logger.record_tabular("actor", ac_num)
            logger.record_tabular("steps", t)
            logger.record_tabular("episodes", num_episodes)
            logger.record_tabular("mean 20 episode reward", mean_20ep_reward)
            # logger.record_tabular("% time spent exploring", int(100 * exploration.value(t)))
            logger.dump_tabular()
