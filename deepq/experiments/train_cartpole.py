import gym

from baselines import deepq


def callback(lcl, _glb):
    # stop training if reward exceeds 199
    is_solved = lcl['t'] > 100 and sum(lcl['episode_rewards'][-101:-1]) / 100 >= 199
    return is_solved


def main():
    env = gym.make("CartPole-v0")
    model = deepq.models.mlp([64])
    act = deepq.learn(
        env,
        q_func=model,
        lr=1e-3,
        max_timesteps=100000,
        buffer_size=50000,
        exploration_fraction=0.1,  # 0.1 越大探索率衰减越低
        exploration_final_eps=0.05,  # 0.02
        print_freq=10,
        prioritized_replay=True,  # yx_add
        param_noise=False,  # yx_add
        callback=callback
    )
    print("Saving model to cartpole_model.pkl")
    act.save("cartpole_model.pkl")


# 如果该文件被当作module,则其__name__则为'train_cartpole',那么这里就不会被执行
# 如果直接执行该文件,那么其__name__为'__main__',main()就会被执行
if __name__ == '__main__':
    main()
