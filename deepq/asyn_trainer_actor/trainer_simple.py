import os
import tempfile
import time

import tensorflow as tf
import numpy as np

import baselines.common.tf_util as U
from baselines.common.tf_util import load_state, save_state
from baselines import logger
# from baselines.common.schedules import LinearSchedule
# from baselines.common.input import observation_input

from baselines import deepq
# from baselines.deepq.replay_buffer import ReplayBuffer, PrioritizedReplayBuffer
from baselines.deepq.utils import ObservationInput
from deepq.simple import ActWrapper
from deepq.asyn_trainer_actor.trainer_build import build_train
from deepq.asyn_trainer_actor.mem_buffer_thread import MemBufferThread


def learn(
          update_flag,
          end_train_flag,
          total_step,
          net_list,
          net_list_lock,
          mem_queue,
          env,
          q_func,
          lr=5e-4,
          max_timesteps=1000000,
          buffer_size=100000,
          batch_size=32,
          checkpoint_freq=10000,
          checkpoint_path=None,
          learning_starts=5000,
          gamma=1.0,
          target_network_update_freq=500,  # asyn中 trainer要比正常运行快,这些参数都有待商议
          actor_network_update_freq=500,  # 最好比actor那边小点(到也没必要,trainer这边运行速度肯定比actor快得多)
          prioritized_replay=False,
          prioritized_replay_alpha=0.6,
          prioritized_replay_beta0=0.4,
          prioritized_replay_beta_iters=None,
          prioritized_replay_eps=1e-6,
          param_noise=False,
          callback=None):
    """Train a deepq model.

    Parameters
    -------
    env: gym.Env
        environment to train on
    q_func: (tf.Variable, int, str, bool) -> tf.Variable
        the model that takes the following inputs:
            observation_in: object
                the output of observation placeholder
            num_actions: int
                number of actions
            scope: str
            reuse: bool
                should be passed to outer variable scope
        and returns a tensor of shape (batch_size, num_actions) with values of every action.
    lr: float
        learning rate for adam optimizer
    max_timesteps: int
        number of env steps to optimizer for
    buffer_size: int
        size of the replay buffer
    exploration_fraction: float
        fraction of entire training period over which the exploration rate is annealed
    exploration_final_eps: float
        final value of random action probability
    batch_size: int
        size of a batched sampled from replay buffer for training
    checkpoint_freq: int
        how often to save the model. This is so that the best version is restored
        at the end of the training. If you do not wish to restore the best version at
        the end of the training set this variable to None.
    learning_starts: int
        how many steps of the model to collect transitions for before learning starts
        asyn 之下该参数修改为在replay_buffer的数据大小下开始？
    gamma: float
        discount factor
    target_network_update_freq: int
        update the target network every `target_network_update_freq` steps.
    prioritized_replay: True
        if True prioritized replay buffer will be used.
    prioritized_replay_alpha: float
        alpha parameter for prioritized replay buffer
    prioritized_replay_beta0: float
        initial value of beta for prioritized replay buffer
    prioritized_replay_beta_iters: int
        number of iterations over which beta will be annealed from initial value
        to 1.0. If set to None equals to max_timesteps.
    prioritized_replay_eps: float
        epsilon to add to the TD errors when updating priorities.
    callback: (locals, globals) -> None
        function called at every steps with state of the algorithm.
        If callback returns true training stops.

    Returns
    -------
    act: ActWrapper
        Wrapper over act function. Adds ability to save it and load it.
        See header of baselines/deepq/categorical.py for details on the act function.
    """
    # Create all the functions necessary to train the model

    # sess = tf.Session()
    config = tf.ConfigProto()
    config.gpu_options.per_process_gpu_memory_fraction = 0.2  # 占用GPU20%的显存
    sess = tf.Session(config=config)
    # sess = U.single_threaded_session()  # 限制使用单核心
    sess.__enter__()

    # capture the shape outside the closure so that the env object is not serialized
    # by cloudpickle when serializing make_obs_ph

    def make_obs_ph(name):
        return ObservationInput(env.observation_space, name=name)

    act, train, update_target,  init_actor_qfunc, update_actor_qfunc, debug = build_train(
        make_obs_ph=make_obs_ph,
        q_func=q_func,
        num_actions=env.action_space.n,
        optimizer=tf.train.AdamOptimizer(learning_rate=lr),
        gamma=gamma,
        grad_norm_clipping=10,
        param_noise=param_noise
    )

    act_params = {
        'make_obs_ph': make_obs_ph,
        'q_func': q_func,
        'num_actions': env.action_space.n,
    }

    act = ActWrapper(act, act_params)

    # Create the replay buffer
    replay_buffer = MemBufferThread(
        mem_queue,
        max_timesteps=max_timesteps,
        buffer_size=buffer_size,
        batch_size=batch_size,
        prioritized_replay=prioritized_replay,
        prioritized_replay_alpha=prioritized_replay_alpha,
        prioritized_replay_beta0=prioritized_replay_beta0,
        prioritized_replay_beta_iters=prioritized_replay_beta_iters,
        prioritized_replay_eps=prioritized_replay_eps
    )

    replay_buffer.setDaemon(True)  # 设置子线程与主线程一起退出,需在start之前
    replay_buffer.start()

    # Initialize the parameters and copy them to the target network.
    U.initialize()
    update_target()
    init_actor_qfunc(sess=sess, net_list=net_list)  # 初始化结束后,先为actor传递一次网络
    # update_actor_qfunc(sess=sess, net_list=net_list, net_list_lock=net_list_lock)
    update_flag.value += 1  # 设置标志位,允许各actor复制初始网络

    with tempfile.TemporaryDirectory() as td:
        td = checkpoint_path or td
        model_file = os.path.join(td, "model_tn")  # 将两端路径名/文件名 合在一起
        model_saved = False
        if tf.train.latest_checkpoint(td) is not None:
            load_state(model_file)
            logger.log('Loaded model from {}'.format(model_file))
            model_saved = True

        t = 0
        # 在最大步数内训练, infinite
        # for t in range(max_timesteps):
        while True:
            if callback is not None:
                if callback(locals(), globals()):
                    break

            # 一直等待replay_buffer的数据足够多,才开始训练网络
            while replay_buffer.__len__() < learning_starts:
                # print(replay_buffer.__len__())
                time.sleep(1)

            # Minimize the error in Bellman's equation on a batch sampled from replay buffer.
            obses_t, actions, rewards, obses_tp1, dones, weights = replay_buffer.sample(total_step.value)

            td_errors = train(obses_t, actions, rewards, obses_tp1, dones, weights)
            # print(td_errors)
            if prioritized_replay:
                replay_buffer.update_priorities(td_errors)

            if t % target_network_update_freq == 0:
                # Update target network periodically.
                update_target()

                # 更新actor_network
            if t % actor_network_update_freq == 0:
                update_actor_qfunc(sess=sess, net_list=net_list, net_list_lock=net_list_lock)

            # time.sleep(0.05)  # 不应该存在
            # checkpoint_freq轮数保存模型
            if (checkpoint_freq is not None
                    and t % checkpoint_freq == 0):
                logger.log("Saving model")
                save_state(model_file)  # 这里是tensorflow的保存方式,是为了继续训练的
                model_saved = True
                act.save("n_robot_model.pkl")  # 这里只保存了act相关内容,可以用来检查运行结果
                # act.save("cartpole_model.pkl")  # 这里只保存了act相关内容,可以用来检查运行结果
                # act.save("MountainCar_model.pkl")
            t += 1
            # # 4 是actor数量,　max_timesteps 是每个actor的最大步数,意味着actor训练结束,train随之结束(not work well)
            # if (total_step.value+4)/4 + 1000 >= max_timesteps:
            #     break
            if end_train_flag.value == 4:  # 4 是actor数量
                break
        # 至此,训练结束
        # 返回一个ActWrapper,用来act.save("cartpole_model.pkl")或其它的动作
        print("end training")
        if model_saved:
            # logger.log("Restored model with mean reward: {}".format(saved_mean_reward))
            logger.log("Restored model")
            load_state(model_file)
    # replay_buffer.join()
    return act
