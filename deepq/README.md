
包含了一些Baselines提供的DQN算法/训练/测试样例, 以及个人修改的模型. 机器人模型的描述文件 xacro、 XML 等, 封装成为符合 gym 的强化学习环境, 和一些额外的强化学习算法实现等.


目录组织如下:
- [original](./original) 包含了Baselines提供的DQN算法, 个人对其中重要部分进行了注解.
- [experiments](./experiments) 中一部分为Baselines的训练测试样例, 还包括了一部分个人写的测试样例, 以及一些训练好的模型.
- [asyn_trainer_actor](./asyn_trainer_actor) 接近介绍中体现的模型, 包括Trainer/Actor/MemBuffer等.
- [asyn_sec](./asyn_sec) 一些精简过的模型, 还有多Actor单Trainer的同步训练模型.

该仓库的代码基于Ubuntu 16.04，ROS Kinetic Kame，gazebo 7.0.0, tensorflow 1.8


比较混乱.

其实已经整理过一次了, 但还是很乱, 早应该使用版本控制工具 :)

该项目由于不可抗力未能完成 : )
