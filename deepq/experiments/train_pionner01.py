# 测试训练单个robot

from baselines import deepq
from baselines.common import set_global_seeds
from baselines import bench
import argparse
from baselines import logger
import gym
# from baselines.common.atari_wrappers import make_atari


def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--env', help='environment ID', default='GazeboPionnerWrapper-v0')
    parser.add_argument('--seed', help='RNG seed', type=int, default=0)
    parser.add_argument('--prioritized', type=int, default=1)
    parser.add_argument('--prioritized-replay-alpha', type=float, default=0.6)
    parser.add_argument('--dueling', type=int, default=1)
    parser.add_argument('--num-timesteps', type=int, default=int(10e5))  # 10e6
    parser.add_argument('--checkpoint-freq', type=int, default=10000)
    # parser.add_argument('--checkpoint-path', type=str, default=None)
    parser.add_argument('--checkpoint-path', type=str, default='/home/yangxu/PycharmProjects/ros_inwork/baselines/deepq/experiments/pionner_save')

    args = parser.parse_args()
    logger.configure()
    set_global_seeds(args.seed)
    env = gym.make(args.env)
    # env = bench.Monitor(env, logger.get_dir()) 
    # env = deepq.wrap_atari_dqn(env)

    # model = deepq.models.mlp([512, 512], [256], dueling=bool(args.dueling), layer_norm=True)  # [512, 512, 512]
    model = deepq.models.mlp([512, 512, 512], layer_norm=True)  # [512, 512, 512]

    deepq.learn(
        env,
        q_func=model,
        lr=1e-3,
        max_timesteps=args.num_timesteps,
        buffer_size=50000,  # 50000
        exploration_fraction=0.1,
        exploration_final_eps=0.01,
        train_freq=1,
        print_freq=10,
        learning_starts=5000,  # 10000
        target_network_update_freq=1000,
        gamma=0.99,
        prioritized_replay=bool(args.prioritized),
        prioritized_replay_alpha=args.prioritized_replay_alpha,
        checkpoint_freq=args.checkpoint_freq,
        checkpoint_path=args.checkpoint_path,
        param_noise=False
    )

    env.close()


if __name__ == '__main__':
    main()
