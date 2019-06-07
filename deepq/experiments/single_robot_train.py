import gym

from baselines import deepq


def main():
    env = gym.make("SingleRobotGpw-v0")
    model = deepq.models.mlp([256, 256, 128])  # [128, 128, 64]
    act = deepq.learn(
        env,
        q_func=model,
        lr=1e-4,
        max_timesteps=1000000,
        buffer_size=300000,
        exploration_fraction=0.20,  # 0.1 表示max_step的0.1处衰减至最低探索率 0.2
        exploration_final_eps=0.02,  # 0.02 表示最低探索率为2% 0.05
        print_freq=20,
        prioritized_replay=True,  # yx_add
        param_noise=False,  # yx_add
        # checkpoint_freq=10000,
        # checkpoint_path='/home/yangxu/PycharmProjects/npc_drl_gpu_ros/baselines/deepq/experiments/pionner_save'
    )
    print("Saving model to single_robot_model.pkl")
    act.save("single_robot_model.pkl")


# 如果该文件被当作module,则其__name__则为'train_cartpole',那么这里就不会被执行
# 如果直接执行该文件,那么其__name__为'__main__',main()就会被执行
if __name__ == '__main__':
    main()
