import os
import tempfile

import tensorflow as tf
import zipfile
import cloudpickle
import numpy as np

import baselines.common.tf_util as U
from baselines.common.tf_util import load_state, save_state
from baselines import logger
from baselines.common.schedules import LinearSchedule
from baselines.common.input import observation_input

from baselines import deepq
from baselines.deepq.replay_buffer import ReplayBuffer, PrioritizedReplayBuffer
from baselines.deepq.utils import ObservationInput


class ActWrapper(object):
    def __init__(self, act, act_params):
        self._act = act
        self._act_params = act_params

    @staticmethod
    def load(path):
        with open(path, "rb") as f:
            model_data, act_params = cloudpickle.load(f)
        act = deepq.build_act(**act_params)
        sess = tf.Session()
        sess.__enter__()
        with tempfile.TemporaryDirectory() as td:
            arc_path = os.path.join(td, "packed.zip")
            with open(arc_path, "wb") as f:
                f.write(model_data)

            zipfile.ZipFile(arc_path, 'r', zipfile.ZIP_DEFLATED).extractall(td)
            load_state(os.path.join(td, "model"))

        return ActWrapper(act, act_params)

    def __call__(self, *args, **kwargs):
        return self._act(*args, **kwargs)

    def save(self, path=None):
        """Save model to a pickle located at `path`"""
        if path is None:
            path = os.path.join(logger.get_dir(), "model.pkl")

        with tempfile.TemporaryDirectory() as td:
            save_state(os.path.join(td, "model"))
            arc_name = os.path.join(td, "packed.zip")
            with zipfile.ZipFile(arc_name, 'w') as zipf:
                for root, dirs, files in os.walk(td):
                    for fname in files:
                        file_path = os.path.join(root, fname)
                        if file_path != arc_name:
                            zipf.write(file_path, os.path.relpath(file_path, td))
            with open(arc_name, "rb") as f:
                model_data = f.read()
        with open(path, "wb") as f:
            cloudpickle.dump((model_data, self._act_params), f)


def load(path):
    """Load act function that was returned by learn function.

    Parameters
    ----------
    path: str
        path to the act function pickle

    Returns
    -------
    act: ActWrapper
        function that takes a batch of observations
        and returns actions.
    """
    return ActWrapper.load(path)


def learn(
          env,
          actor_deque,
          action_pipes,
          q_func,
          lr=5e-4,
          max_timesteps=100000,
          buffer_size=50000,
          exploration_fraction=0.1,
          exploration_final_eps=0.02,
          train_freq=1,
          batch_size=32,
          print_freq=100,
          checkpoint_freq=10000,
          checkpoint_path=None,
          learning_starts=1000,
          gamma=1.0,
          target_network_update_freq=500,
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
    actor_deque: structure is --> (ac_num, obs, action, new_obs, rew, done)
    action_pipes: structure is --> pipes_conn1 = [pipes[i][1] for i in range(0, 2)]
        use --> action_pipes[actor_num].send(s)  default is str
        至于为什么一处为deque,一处为pipe. well, actor需要接受action来执行下一步,此前为阻塞状态.
        而trainer是响应式的,无论哪个actor有数据都要进行计算,使用deque.empty()很方便,
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
    train_freq: int
        update the model every `train_freq` steps.
        set to None to disable printing
    batch_size: int
        size of a batched sampled from replay buffer for training
    print_freq: int
        how often to print out training progress
        set to None to disable printing
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

    sess = tf.Session()
    sess.__enter__()

    # capture the shape outside the closure so that the env object is not serialized
    # by cloudpickle when serializing make_obs_ph

    def make_obs_ph(name):
        return ObservationInput(env.observation_space, name=name)

    act, train, update_target, debug = deepq.build_train(
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
    if prioritized_replay:
        replay_buffer = PrioritizedReplayBuffer(buffer_size, alpha=prioritized_replay_alpha)
        if prioritized_replay_beta_iters is None:
            prioritized_replay_beta_iters = max_timesteps
        beta_schedule = LinearSchedule(prioritized_replay_beta_iters,
                                       initial_p=prioritized_replay_beta0,
                                       final_p=1.0)
    else:
        replay_buffer = ReplayBuffer(buffer_size)
        beta_schedule = None
    # Create the schedule for exploration starting from 1.　探索率
    exploration = LinearSchedule(schedule_timesteps=int(exploration_fraction * max_timesteps),
                                 initial_p=1.0,
                                 final_p=exploration_final_eps)

    # Initialize the parameters and copy them to the target network.
    U.initialize()
    update_target()

    episode_rewards = [0.0]
    saved_mean_reward = None
    # obs = env.reset()
    reset = True
    done = None
    end = 100  # 传输一个非正常动作,结束训练

    with tempfile.TemporaryDirectory() as td:
        td = checkpoint_path or td
        model_file = os.path.join(td, "model_tn")
        model_saved = False
        if tf.train.latest_checkpoint(td) is not None:
            load_state(model_file)
            logger.log('Loaded model from {}'.format(model_file))
            model_saved = True

        # 在最大步数内训练
        t = 0
        while t <= max_timesteps:
            if callback is not None:
                if callback(locals(), globals()):
                    break
            if actor_deque.empty() is True:
                pass
                # time.sleep()
            else:
                actor_information = actor_deque.get()
                if actor_information[2] is None:  # 表示其为一轮开始
                    ac_num = actor_information[0]
                    new_obs = actor_information[3]
                    done = False  # important
                    # print("ac_num "+str(ac_num)+" start")
                else:
                    ac_num = actor_information[0]
                    obs = actor_information[1]
                    action = actor_information[2]
                    new_obs = actor_information[3]
                    rew = actor_information[4]
                    done = actor_information[5]
                    replay_buffer.add(obs, action, rew, new_obs, float(done))
                if done:  # done 与start是不会共存的
                    # obs = env.reset()
                    # episode_rewards.append(0.0)
                    reset = True
                    # print("ac_num " + str(ac_num) + " done")
                else:
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
                    action = act(np.array(new_obs)[None], update_eps=update_eps, **kwargs)[0]
                    env_action = action
                    reset = False
                    action_pipes[ac_num-1].send(env_action)  # 这里ac_num与pipe位置没有对齐
                    # 经过learning_starts步后开始训练网络（先在buffer中存入一定量数据）
                    # 每经过train_freq步进行一次梯度下降
                    if t > learning_starts and t % train_freq == 0:
                        # Minimize the error in Bellman's equation on a batch sampled from replay buffer.
                        if prioritized_replay:
                            experience = replay_buffer.sample(batch_size, beta=beta_schedule.value(t))  # 注意beta的用法
                            (obses_t, actions, rewards, obses_tp1, dones, weights, batch_idxes) = experience
                        else:
                            obses_t, actions, rewards, obses_tp1, dones = replay_buffer.sample(batch_size)
                            #  np.ones_like() : Return an array of ones with the same shape and type as a given array.
                            weights, batch_idxes = np.ones_like(rewards), None
                        td_errors = train(obses_t, actions, rewards, obses_tp1, dones, weights)
                        # print(td_errors)
                        if prioritized_replay:
                            new_priorities = np.abs(td_errors) + prioritized_replay_eps
                            replay_buffer.update_priorities(batch_idxes, new_priorities)

                    if t > learning_starts and t % target_network_update_freq == 0:
                        # Update target network periodically.
                        update_target()

                # 下面是关于输出训练信息,以及保存网络参数的部分
                if print_freq is not None and t % print_freq == 0:
                    logger.record_tabular("total_steps", t)
                    # logger.record_tabular("episodes", num_episodes)
                    # logger.record_tabular("mean 20 episode reward", mean_100ep_reward)
                    logger.record_tabular("% time spent exploring", int(100 * exploration.value(t)))
                    logger.dump_tabular()

                # checkpoint_freq轮数、mean reward增长才会保存模型
                if checkpoint_freq is not None and t > learning_starts and t % checkpoint_freq == 0:
                    save_state(model_file)
                    model_saved = True
                t += 1
        # 至此,训练结束
        # end = True
        for i in range(0, len(action_pipes)):
            action_pipes[i].send(end)  # end = 100
        # 训练结束后保存最佳模型
        # if model_saved:
        #     if print_freq is not None:
        #         logger.log("Restored model with mean reward: {}".format(saved_mean_reward))
        #     load_state(model_file)
        # 返回一个ActWrapper,用来act.save("cartpole_model.pkl")或其它的动作
    return act
