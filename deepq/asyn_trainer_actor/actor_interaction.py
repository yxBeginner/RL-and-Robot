# yx 2018.07.04
import time
import tensorflow as tf
import numpy as np

from baselines import logger
from baselines.common.schedules import LinearSchedule

from baselines.deepq.utils import ObservationInput
from deepq.asyn_trainer_actor.actor_build import actor_build
from baselines.deepq.simple import ActWrapper
import baselines.common.tf_util as U

# import os
# os.environ["CUDA_VISIBLE_DEVICES"] = "-1"  # disable gpu
# from memory_profiler import profile
import sys


# @profile
def actor_inter(update_flag,
                end_train_flag,
                total_step,
                net_list,
                net_list_lock,
                mem_queue,
                actor_num,
                env,
                q_func,
                actor_max_timesteps=100000,
                # mem_buffer_size=5000,  # 目前每次step都发送
                exploration_fraction=0.1,
                exploration_final_eps=0.02,
                print_freq=100,
                actor_network_update_freq=500,
                update_starts=6000,
                param_noise=False,
                callback=None):
    # actor_num  # actor 的序号
    # mem_buffer_size=50000,  # 多久向trainer_buffer发送一次memory
    # actor_network_update_freq=500,  # actor_network 更新频率
    # update_starts=6000,  # 需要设置actor从多少步之后开始从trainer复制网络（trainer则需要在buffer一定数量之后才开始训练）
    multi_step_num = 3  # multi step return 10, 5
    gamma = 0.5  # 折扣率

    # 使用gpu增长占用显存,无法正确启动程序
    # config = tf.ConfigProto()
    # config.gpu_options.allow_growth = True
    # sess = tf.Session(config=config)
    # sess = tf.Session(config=tf.ConfigProto(device_count={'gpu': 0}))
    config = tf.ConfigProto()
    config.gpu_options.per_process_gpu_memory_fraction = 0.1  # 占用GPU 10%的显存
    sess = tf.Session(config=config)
    # sess = tf.Session()
    sess.__enter__()

    # capture the shape outside the closure so that the env object is not serialized
    # by cloudpickle when serializing make_obs_ph

    # 等待trainer完成网络初始化,该步骤应在actor_build()之前,因为要使用net_list来setup placeholder
    while bool(update_flag.value) is not True:
        time.sleep(0.01)

    def make_obs_ph(name):
        return ObservationInput(env.observation_space, name=name)

    act, update_qfunc= actor_build(make_obs_ph=make_obs_ph, q_func=q_func,net_list=net_list,
                                   num_actions=env.action_space.n, param_noise=param_noise)

    act_params = {
        'make_obs_ph': make_obs_ph,
        'q_func': q_func,
        'num_actions': env.action_space.n,
    }

    act = ActWrapper(act, act_params)  # ActWrapper() 可能根本不需要

    # 设置本actor的replay_buffer
    # Create the replay buffer
    # if prioritized_replay:
    #     replay_buffer = PrioritizedReplayBuffer(buffer_size, alpha=prioritized_replay_alpha)
    #     if prioritized_replay_beta_iters is None:
    #         prioritized_replay_beta_iters = max_timesteps
    #     beta_schedule = LinearSchedule(prioritized_replay_beta_iters,
    #                                    initial_p=prioritized_replay_beta0,
    #                                    final_p=1.0)
    # else:
    #     replay_buffer = ReplayBuffer(buffer_size)
    #     beta_schedule = None

    # Create the schedule for exploration starting from 1.(探索率,各robot不统一)
    exploration = LinearSchedule(schedule_timesteps=int(exploration_fraction * actor_max_timesteps),
                                 initial_p=1.0,
                                 final_p=exploration_final_eps)

    # Initialize the parameters and copy the qfunc network.
    U.initialize()  # 必要
    # 此时,trainer的q network 是随机生成的,但为保证一致性,各actor仍要复制该网络
    # update_qfunc(sess=sess, net_list=net_list, net_list_lock=net_list_lock)
    update_qfunc(sess=sess, net_list_lock=net_list_lock)

    episode_rewards = [0.0]
    obs = env.reset()
    reset = True

    # 在最大步数内interact
    for t in range(actor_max_timesteps):
        # if callback is not None:
        #     if callback(locals(), globals()):
        #         break
        # Take action and update exploration to the newest value
        kwargs = {}
        if not param_noise:
            update_eps = exploration.value(t)
            update_param_noise_threshold = 0.
        else:
            update_eps = 0.

            update_param_noise_threshold = -np.log(1. - exploration.value(t) + exploration.value(t) / float(env.action_space.n))
            kwargs['reset'] = reset
            kwargs['update_param_noise_threshold'] = update_param_noise_threshold
            kwargs['update_param_noise_scale'] = True
        # 这里选择动作
        action = act(np.array(obs)[None], update_eps=update_eps, **kwargs)[0]
        env_action = action
        reset = False
        new_obs, rew, done, _ = env.step(env_action)
        # 将交互的记忆添加至queue
        single_mem = (obs, action, rew, new_obs, float(done))
        if mem_queue.full() is not True:
            mem_queue.put(single_mem)  # trainer记得从元组中取出
        obs = new_obs

        total_step.value += 1  # 更新全局steps
        # 将每个episode即时回报累加,在每个actor上单独计算（better）,
        episode_rewards[-1] += rew
        if done:
            # # 类似倒垂摆问题,每个episode 的step数量较少,比较好的方法应该是done之后更新
            if t > update_starts:
                # update_qfunc(sess=sess, net_list=net_list, net_list_lock=net_list_lock)
                update_qfunc(sess=sess, net_list_lock=net_list_lock)
            obs = env.reset()
            episode_rewards.append(0.0)
            reset = True
            # capital_t = 1000 + multi_step_num + 1

        # 从trainer更新network的参数,由于使用多进程,q_func不能够去每个step都更新,那么这个频率就和环境关系比较大,像倒垂摆问题,每个
        # episode 的step数量较少,比较好的方法应该是done之后更新,而robot navigation step比较多,更好是每隔一定步数更新
        # if t > update_starts and t % actor_network_update_freq == 0:
        #     # update_qfunc(sess=sess, net_list=net_list, net_list_lock=net_list_lock)
        #     update_qfunc(sess=sess, net_list_index=net_list_index)

        mean_100ep_reward = round(np.mean(episode_rewards[-101:-1]), 1)  # round(np.mean(episode_rewards[-21:-1]), 1)
        num_episodes = len(episode_rewards)
        # 下面关于输出训练信息
        if done and print_freq is not None and len(episode_rewards) % print_freq == 0:
            logger.record_tabular("actor-", actor_num)
            logger.record_tabular("steps", t)
            logger.record_tabular("episodes", num_episodes)
            logger.record_tabular("mean 100 episode reward", mean_100ep_reward)
            logger.record_tabular("% time spent exploring", int(100 * exploration.value(t)))
            logger.record_tabular("total_steps", total_step.value)
            logger.dump_tabular()

        if t == actor_max_timesteps-1:
            end_train_flag.value += 1

        # for debug
        # if t % 500 == 0:
        #     # print('net_list: ')
        #     # print(net_list)
        #     print('single_mem: ')
        #     print(single_mem)
        #     print('net_list size: ' + str(sys.getsizeof(net_list)))
        #     print('single_mem size: ' + str(sys.getsizeof(single_mem)))
        #     print('action size: ' + str(sys.getsizeof(action)))

    # yx: 个人编写的multi step return version, 在经典RL环境下训练良好, 前期比原算法得分提升快得多
    # 通常情况下达到预设分数也比单步回报要快, 但是似乎更不稳定, 有时分数波动非常剧烈
    t = 0
    # 由于gym环境实现的问题, i_episode, episode_t皆需要与具体环境相结合
    # for i_episode in range(int(actor_max_timesteps/1000)):
    for i_episode in range(int(actor_max_timesteps/10)):
        action_rewards_list = [0 for x in range(1500)]
        obs_list = list()
        obs_list.append(obs)  # 将S(0)加入序列
        action_list = []
        capital_t = 1500 + multi_step_num + 1  # for multi step return, can be infinite

        for episode_t in range(1520):  # 注意这里的episode_t 将不是正常的步数,done之后仍然要运行,并计算g_reward
            g_reward = 0.0
            if episode_t < capital_t:  # 与环境交互的部分,done之后就不会运行到
                # Take action and update exploration to the newest value
                kwargs = {}
                if not param_noise:
                    update_eps = exploration.value(t)
                    update_param_noise_threshold = 0.
                else:
                    update_eps = 0.
                    update_param_noise_threshold = -np.log(
                        1. - exploration.value(t) + exploration.value(t) / float(env.action_space.n))
                    kwargs['reset'] = reset
                    kwargs['update_param_noise_threshold'] = update_param_noise_threshold
                    kwargs['update_param_noise_scale'] = True
                # 这里选择动作
                action = act(np.array(obs)[None], update_eps=update_eps, **kwargs)[0]
                env_action = action
                reset = False
                # print("action: "+str(env_action))
                new_obs, rew, done, _ = env.step(env_action)
                action_rewards_list[episode_t] = rew  # reward 存储入reward list
                obs_list.append(new_obs)  # obs 的list
                action_list.append(action)
                obs = new_obs
                if done:
                    capital_t = episode_t + 1
                # 将每个episode即时回报累加,在每个actor上单独计算（better）,
                episode_rewards[-1] += rew
                t += 1
                total_step.value += 1  # 更新全局steps
                # # 从trainer更新network的参数
                # if t > update_starts and t % actor_network_update_freq == 0:
                #     # update_qfunc(sess=sess, net_list=net_list, net_list_lock=net_list_lock)
                #     update_qfunc(sess=sess, net_list_lock=net_list_lock)

            tau = episode_t - multi_step_num + 1
            if tau >= 0:
                min_step = min(tau+multi_step_num, capital_t)
                for i in range(tau+1, min_step):
                    g_reward += (gamma**(i-tau-1)) * action_rewards_list[i]
                # mem 加入　S(tau+n),最后几步中done一直为True,那么trainer的td目标不会加maxQ(s,a),
                if tau+multi_step_num < capital_t:
                    single_mem = (obs_list[tau], action_list[tau], g_reward, obs_list[tau+multi_step_num], float(done))
                else:
                    # 这里的S(tau+n)已经不重要了,trainer不会实际用到
                    done = 1
                    single_mem = (obs_list[tau], action_list[tau], g_reward, obs_list[-1], float(done))
                if mem_queue.full() is not True:
                    mem_queue.put(single_mem)  # trainer记得从元组中取出
            # print('actor -- ' + str(actor_num) + '  episode_t -- ' + str(episode_t))
            # print('actor -- ' + str(actor_num) + '  capital_t -- ' + str(capital_t))
            # print('actor -- ' + str(actor_num) + '  tau -- ' + str(tau))

            if t == actor_max_timesteps - 1:
                end_train_flag.value += 1
                break
            if tau == capital_t-1:
                break
        # end for    一步之内
        if t > update_starts:
            # update_qfunc(sess=sess, net_list=net_list, net_list_lock=net_list_lock)
            update_qfunc(sess=sess, net_list_lock=net_list_lock)
        obs = env.reset()
        episode_rewards.append(0.0)
        reset = True

        if t == actor_max_timesteps - 1:
            break

        # 平均100次情景的回报（）, 注意episode是轮数,不是步数
        mean_100ep_reward = round(np.mean(episode_rewards[-101:-1]), 1)  # round(np.mean(episode_rewards[-101:-1]), 1)
        num_episodes = len(episode_rewards)
        # 下面关于输出训练信息
        if print_freq is not None and len(episode_rewards) % print_freq == 0:
            logger.record_tabular("actor-", actor_num)
            logger.record_tabular("steps", t)
            logger.record_tabular("episodes", num_episodes)
            logger.record_tabular("mean 100 episode reward", mean_100ep_reward)  # reward还是正常的,不是g_reward
            logger.record_tabular("% time spent exploring", int(100 * exploration.value(t)))
            logger.record_tabular("total_steps", total_step.value)
            logger.dump_tabular()


