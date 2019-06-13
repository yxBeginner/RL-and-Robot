# Deep Reinforcement Learning and Path Planning

### 简介

包含了一些Baselines提供的DQN算法/训练/测试样例, 以及个人修改的模型. 机器人模型的描述文件 xacro、 XML 等, 封装成为符合 gym 的强化学习环境, 和一些额外的强化学习算法实现等. 关于这个项目的模型, 更多的介绍位于[deepq](./deepq)中.

### 目录结构组织
* [deepq](./deepq) 主要是`异步 Deep Q Learning`模型.
  - [original](./deepq/original) 包含了Baselines提供的DQN算法, 个人对其中重要部分进行了注解.
  - [experiments](./deepq/experiments) 中一部分为Baselines的训练测试样例, 还包括了一部分个人写的测试样例, 以及一些训练好的模型.
  - [asyn_trainer_actor](./deepq/asyn_trainer_actor) 接近介绍中体现的模型, 包括Trainer/Actor/MemBuffer等.
  - [asyn_sec](./deepq/asyn_sec) 一些精简过的模型, 还有多Actor单Trainer的同步训练模型, 其中包含了训练好的模型(`.pkl`).
* [ppo2](./ppo2) `Proximal Policy Optimization`（近端策略优化), Baselines的代码, 个人对一些关键部分进行了注释.
* [p2os_test](./p2os_test) 主要是对强化学习环境的封装. 其中包含了诸多版本, 每种环境的定义都或多或少有一些不同之处, 包括状态空间/动作空间/奖赏函数的设计/命令传递/数据服务的处理等, 在每个版本的开始部分都进行了简单的介绍. 启动环境后, 会到ROS core注册监听相关的数据服务, 所有操作符合ROS的流程.
* [p2os_urdf](./p2os_urdf) 修动后的机器人模型与模拟世界地图等, 其它部分来自[p2os](http://wiki.ros.org/p2os), 但这是为`groovy`版本设计的, 有些东西在`Kinetic Kame`下不能正常运行, 该目录下对其进行了修改. 添加了激光模型, 修改了3d模型的位置, 仿真定位, 差速控制等.

### 运行环境
该仓库的代码运行环境为 `Ubuntu 16.04`，`ROS Kinetic Kame`，`gazebo 7.0.0`, `tensorflow 1.8` . 单机情景.
### TODO(yx):
- [ ] 为了方便理解代码以及修改, 代码中留下了大量的注释, 所以部分代码会显得有些凌乱.
- [ ] 代码结构缺少整理, 需要进一步完善.
- [ ] 部分包的位置被重新安排, 但是import并未修改, 需要重新导入.

*该项目由于不可抗力, 最终未能完成 : )*
*开发过程中有很多被否定的方案, 但有一些仍然有价值, 全部都放在这里, 留作参考.*

