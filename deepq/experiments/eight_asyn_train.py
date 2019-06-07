import argparse
from multiprocessing import Process, Manager, Lock, Queue, Value

from baselines import deepq
from baselines.common import set_global_seeds
from baselines import logger
import gym
from deepq.asyn_trainer_actor.actor_interaction import actor_inter
from deepq.asyn_trainer_actor.trainer_simple import learn
from deepq.asyn_trainer_actor.new_models import mlp
from p2os_test.src.eight_asyn_gpw import set_gpw_num
# from p2os_test.src.set_actor_num import set_gpw_num
# from baselines.common.atari_wrappers import make_atari


def trainer(lock1, net_list, mem_queue, total_step, end_train_flag, update_flag):
    # 可以为learn过程设置终止条件
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--env', help='environment ID', default='GpwTrainer-v0')
    parser.add_argument('--seed', help='RNG seed', type=int, default=0)
    parser.add_argument('--prioritized', type=int, default=0)
    parser.add_argument('--prioritized-replay-alpha', type=float, default=0.6)
    parser.add_argument('--dueling', type=int, default=1)
    parser.add_argument('--num-timesteps', type=int, default=int(10e7))  # 10e6
    # parser.add_argument('--checkpoint-freq', type=int, default=None)
    parser.add_argument('--checkpoint-freq', type=int, default=100000)  # frep与path都需激活
    # parser.add_argument('--checkpoint-path', type=str, default=None)
    parser.add_argument('--checkpoint-path', type=str, default='/home/yangxu/PycharmProjects/npc_drl_gpu_ros/baselines/deepq/experiments/pionner_save')

    args = parser.parse_args()
    logger.configure()
    set_global_seeds(args.seed)
    env = gym.make(args.env)
    # env = bench.Monitor(env, logger.get_dir())  # 这里的作用暂时没看
    # env = deepq.wrap_atari_dqn(env)
    # 这里使用了3个卷积层,两个全连接层（第一个全连接层为num_convs*hiddens,第二层为hiddens*num_action）
    # 注意卷积层的输入应为矩阵而不是向量,要么将CNN换成mlp,要么用RNN
    # model = deepq.models.cnn_to_mlp(
    #     convs=[(32, 8, 4), (64, 4, 2), (64, 3, 1)],
    #     hiddens=[256],
    #     dueling=bool(args.dueling),
    # )
    # early model [512,512,256], [128]
    model = mlp([256, 256, 128], [64], dueling=bool(args.dueling), layer_norm=True)  # [512, 512, 512]

    learn(
        update_flag=update_flag,
        end_train_flag=end_train_flag,
        total_step=total_step,
        net_list=net_list,
        net_list_lock=lock1,
        mem_queue=mem_queue,
        env=env,
        q_func=model,
        lr=1e-4,  # 1e-3
        max_timesteps=args.num_timesteps,
        buffer_size=1000000,
        learning_starts=50000,  # 10000
        target_network_update_freq=1000,
        actor_network_update_freq=300,
        gamma=0.99,
        prioritized_replay=bool(args.prioritized),
        prioritized_replay_alpha=args.prioritized_replay_alpha,
        checkpoint_freq=args.checkpoint_freq,
        checkpoint_path=args.checkpoint_path,
        param_noise=False
    )

    env.close()


def actor(lock1, net_list, mem_queue, total_step, end_train_flag, update_flag, actor_num):
    acparser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    acparser.add_argument('--acenv', help='environment ID', default='AsynGpwEight-v0')
    acparser.add_argument('--acseed', help='RNG seed', type=int, default=0)
    acparser.add_argument('--acdueling', type=int, default=1)  # 与trainer保持一致
    acparser.add_argument('--acnum-timesteps', type=int, default=int(10e6))  # 10e6

    acargs = acparser.parse_args()
    logger.configure()
    set_global_seeds(acargs.acseed)
    set_gpw_num(actor_num)  # 设置环境号
    acenv = gym.make(acargs.acenv)
    # env = bench.Monitor(env, logger.get_dir())  # 这里的作用暂时没看
    # env = deepq.wrap_atari_dqn(env)
    # 这里使用了3个卷积层,两个全连接层（第一个全连接层为num_convs*hiddens,第二层为hiddens*num_action）
    # 注意卷积层的输入应为矩阵而不是向量,要么将CNN换成mlp,要么用RNN
    # model = deepq.models.cnn_to_mlp(
    #     convs=[(32, 8, 4), (64, 4, 2), (64, 3, 1)],
    #     hiddens=[256],
    #     dueling=bool(args.dueling),
    # )

    # early model [512,512,256], [128]
    model = mlp([256, 256, 128], [64], dueling=bool(acargs.acdueling), layer_norm=True)  # [512, 512, 512]

    actor_inter(
                update_flag=update_flag,
                end_train_flag=end_train_flag,
                total_step=total_step,
                net_list=net_list,
                net_list_lock=lock1,
                mem_queue=mem_queue,
                actor_num=actor_num,
                env=acenv,
                q_func=model,
                actor_max_timesteps=acargs.acnum_timesteps,
                # mem_buffer_size=5000,
                exploration_fraction=0.1,
                exploration_final_eps=0.01,  # 0.02
                print_freq=40,  # 100
                actor_network_update_freq=100,
                update_starts=15000,  # 至少等于trainer中learning_starts,该参数,以便随后复制网络 5000
                param_noise=False,
                callback=None)  # actor可能不需要callback

    acenv.close()


def main():
    lock1 = Lock()
    # lock2 = Lock()
    net_list = Manager().list()
    total_step = Value('i', 0)  # 用于记录所有robot的总episode数目
    update_flag = Value('i', 0)  # 启动更新的标志位
    end_train_flag = Value('i', 0)  # 终止更新的标志位
    # mem_list = Manager().list()
    mem_queue = Queue(maxsize=3000)
    train_process = Process(target=trainer, args=(lock1, net_list, mem_queue, total_step, end_train_flag, update_flag))
    actor_process_01 = Process(target=actor, args=(lock1, net_list, mem_queue, total_step, end_train_flag, update_flag, 1))
    actor_process_02 = Process(target=actor, args=(lock1, net_list, mem_queue, total_step, end_train_flag, update_flag, 2))
    actor_process_03 = Process(target=actor, args=(lock1, net_list, mem_queue, total_step, end_train_flag, update_flag, 3))
    actor_process_04 = Process(target=actor, args=(lock1, net_list, mem_queue, total_step, end_train_flag, update_flag, 4))
    # actor_process_05 = Process(target=actor, args=(lock1, net_list, mem_queue, total_step, end_train_flag, update_flag, 5))
    # actor_process_06 = Process(target=actor, args=(lock1, net_list, mem_queue, total_step, end_train_flag, update_flag, 6))
    # actor_process_07 = Process(target=actor, args=(lock1, net_list, mem_queue, total_step, end_train_flag, update_flag, 7))
    # actor_process_08 = Process(target=actor, args=(lock1, net_list, mem_queue, total_step, end_train_flag, update_flag, 8))

    train_process.start()
    actor_process_01.start()
    actor_process_02.start()
    actor_process_03.start()
    actor_process_04.start()
    # actor_process_05.start()
    # actor_process_06.start()
    # actor_process_07.start()
    # actor_process_08.start()
    train_process.join()
    actor_process_01.join()
    actor_process_02.join()
    actor_process_03.join()
    actor_process_04.join()
    # actor_process_05.join()
    # actor_process_06.join()
    # actor_process_07.join()
    # actor_process_08.join()


if __name__ == '__main__':
    main()
