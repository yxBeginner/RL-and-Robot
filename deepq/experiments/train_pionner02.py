import gym

from baselines import deepq


def main():
    env = gym.make("GazeboPionnerWrapper-v0")
    # Enabling layer_norm here is import for parameter space noise!
    model = deepq.models.mlp([512, 512, 512], layer_norm=True)  # [512, 512, 512]
    # print("test1")
    act = deepq.learn(
        env,
        q_func=model,
        lr=1e-3,
        max_timesteps=1000000,
        buffer_size=50000,
        exploration_fraction=0.1,
        exploration_final_eps=0.1,
        print_freq=10,
        param_noise=False
    )
    print("Saving model to train_pionner02.pkl")
    act.save("train_pionner02.pkl")


if __name__ == '__main__':
    main()
