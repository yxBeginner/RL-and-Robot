from multiprocessing import Process, Manager, Lock, Queue, Value

from baselines import deepq
import gym
from deepq.asyn_trainer_actor.actor_interaction import actor_inter
from deepq.asyn_trainer_actor.trainer_simple import learn
from deepq.asyn_trainer_actor.new_models import mlp
# from p2os_test.src.set_actor_num import set_gpw_num
# from baselines.common.atari_wrappers import make_atari

# 测试用环境

# mem_buffer_thread 中的sleep()记得处理
# 与robot中不同,这里actor的step比trainer更快
# 其实对于cartpole等问题,使用多线程可能更好
def trainer(lock, net_list, mem_queue, total_step, end_train_flag, update_flag):
    # 可以为learn过程设置终止条件
    env = gym.make("CartPole-v0")
    # env = gym.make("MountainCar-v0")
    # model = deepq.models.mlp([64])
    model = mlp([64], [16], dueling=True, layer_norm=True)
    act = learn(
        update_flag=update_flag,
        end_train_flag=end_train_flag,
        total_step=total_step,
        net_list=net_list,
        net_list_lock=lock,
        mem_queue=mem_queue,
        env=env,
        q_func=model,
        lr=1e-4,  # 1e-3
        max_timesteps=100000,  # 这个参数已经没有什么实际用处了
        buffer_size=50000,
        # checkpoint_freq=10000,
        # checkpoint_path='/home/yangxu/PycharmProjects/ros_inwork/baselines/deepq/experiments/pionner_save',
        learning_starts=1000,  # 10000
        target_network_update_freq=500,
        actor_network_update_freq=50,  # 50
        gamma=0.99,
        prioritized_replay=False,
        prioritized_replay_alpha=0.6,
        param_noise=False
    )

    print("Saving model to cartpole_model.pkl")
    act.save("cartpole_model.pkl")
    # act.save("MountainCar_model.pkl")
    env.close()


def actor(lock, net_list, mem_queue, total_step, end_train_flag, update_flag, actor_num):

    acenv = gym.make("CartPole-v0")
    # acenv = gym.make("MountainCar-v0")
    # model = deepq.models.mlp([64])  # 为什么使用普通的模型不能正常工作
    model = mlp([64], [16], dueling=True, layer_norm=True)

    actor_inter(
                update_flag=update_flag,
                end_train_flag=end_train_flag,
                total_step=total_step,
                net_list=net_list,
                net_list_lock=lock,
                mem_queue=mem_queue,
                actor_num=actor_num,
                env=acenv,
                q_func=model,
                actor_max_timesteps=100000,  # 100000
                # mem_buffer_size=5000,
                exploration_fraction=0.1,
                exploration_final_eps=0.02,  # 0.1
                print_freq=20,  # 100
                actor_network_update_freq=50,
                update_starts=500,  # 至少等于trainer中learning_starts,该参数,以便随后复制网络 5000
                param_noise=True,
                callback=None)  # actor可能不需要callback

    acenv.close()


def main():
    lock1 = Lock()
    # lock2 = Lock()
    net_list = Manager().list()

    # index1 = Value('i', 0)  # 不使用Lock(),而是标志位,降低lock竞争,毕竟只有一个进程会修改net_list,实现方式有点trick
    total_step = Value('i', 0)  # 用于记录所有robot的总episode数目
    update_flag = Value('i', 0)  # 启动更新的标志位
    end_train_flag = Value('i', 0)  # 终止更新的标志位
    # mem_list = Manager().list()
    mem_queue = Queue(maxsize=10000)
    train_process = Process(target=trainer, args=(lock1, net_list, mem_queue, total_step, end_train_flag, update_flag))
    actor_process_01 = Process(target=actor, args=(lock1, net_list, mem_queue, total_step, end_train_flag, update_flag, 1))
    actor_process_02 = Process(target=actor, args=(lock1, net_list, mem_queue, total_step, end_train_flag, update_flag, 2))
    actor_process_03 = Process(target=actor, args=(lock1, net_list, mem_queue, total_step, end_train_flag, update_flag, 3))
    actor_process_04 = Process(target=actor, args=(lock1, net_list, mem_queue, total_step, end_train_flag, update_flag, 4))

    train_process.start()
    actor_process_01.start()
    actor_process_02.start()
    actor_process_03.start()
    actor_process_04.start()

    train_process.join()
    actor_process_01.join()
    actor_process_02.join()
    actor_process_03.join()
    actor_process_04.join()


if __name__ == '__main__':
    main()
