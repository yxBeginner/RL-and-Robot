from multiprocessing import Process, Queue, Pipe

from baselines import deepq
import gym
from deepq.asyn_sec.actor_interact_env import actor_inter
from deepq.asyn_sec.simple_multi_agent import learn
# from deepq.asyn_trainer_actor.new_models import mlp
from deepq.models import mlp
# from p2os_test.src.seventh_edition_gpw import set_gpw_num
from p2os_test.src.sixth_edition_gpw import set_gpw_num
from baselines.common import set_global_seeds


def trainer(in_actor_deque, in_action_pipes):
    # 可以为learn过程设置终止条件
    env = gym.make("GpwTrainer-v0")
    # env = gym.make("MountainCar-v0")
    # model = deepq.models.mlp([64])
    model = mlp([256, 256, 128], layer_norm=True)
    act = learn(
        env,
        actor_deque=in_actor_deque,
        action_pipes=in_action_pipes,
        q_func=model,
        lr=1e-4,  # 1e-3
        max_timesteps=5000000,
        buffer_size=1000000,  # 300000
        exploration_fraction=0.30,
        exploration_final_eps=0.05,  # 0.02
        train_freq=1,
        batch_size=32,
        print_freq=30000,  # 这里的print_freq与step相关,而actor处与episode相关
        # checkpoint_freq=10000,
        # checkpoint_path=None,
        learning_starts=1000,
        gamma=1.0,
        target_network_update_freq=500,
        prioritized_replay=True,
        prioritized_replay_alpha=0.6,
        # prioritized_replay_beta0=0.4,
        # param_noise=True,
    )
    print("All end")
    print("Saving model")
    act.save("asyn_rob_model.pkl")
    env.close()


def actor(in_ac_num, in_actor_deque, in_action_pipes):

    set_global_seeds(0)
    set_gpw_num(in_ac_num)  # 设置环境号
    acenv = gym.make("SixthRobGpw-v0")
    # acenv = gym.make("MountainCar-v0")
    actor_inter(env=acenv, ac_num=in_ac_num, actor_deque=in_actor_deque,
                action_pipes=in_action_pipes, print_freq=20)
    acenv.close()


def main():
    pipes = [Pipe(duplex=False) for x in range(0, 4)]
    pipes_conn1 = [pipes[i][1] for i in range(0, 4)]
    actor_inf_queue = Queue(maxsize=5)
    train_process = Process(target=trainer, args=(actor_inf_queue, pipes_conn1))
    actor_process_01 = Process(target=actor, args=(1, actor_inf_queue, pipes[0][0]))
    actor_process_02 = Process(target=actor, args=(2, actor_inf_queue, pipes[1][0]))
    actor_process_03 = Process(target=actor, args=(3, actor_inf_queue, pipes[2][0]))
    actor_process_04 = Process(target=actor, args=(4, actor_inf_queue, pipes[3][0]))
    # actor_process_05 = Process(target=actor, args=(lock1, net_list, mem_queue, total_step, update_flag, 5))
    # actor_process_06 = Process(target=actor, args=(lock1, net_list, mem_queue, total_step, update_flag, 6))
    # actor_process_07 = Process(target=actor, args=(lock1, net_list, mem_queue, total_step, update_flag, 7))
    # actor_process_08 = Process(target=actor, args=(lock1, net_list, mem_queue, total_step, update_flag, 8))

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
