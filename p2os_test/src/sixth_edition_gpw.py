#!/usr/bin/python3
import time
# import threading
import math

import gym
import numpy as np
from gym.spaces import Discrete
from gym.utils import seeding
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState
from std_srvs.srv import Empty
# from gazebo_msgs.srv import GetModelStateResponse
# from rosgraph_msgs.msg import Clock

enable_correction = True
ac_num = 0


def set_gpw_num(num):
    global ac_num
    ac_num = num

# 连续状态空间
#


# 由于先前的写法,这里暂时只能单独写,而不能统一处理
def reset_node_init():
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

    new_obstacle_state = ModelState()
    new_obstacle_state.model_name = "robot"+str(ac_num)
    # cc_x = 0.0  # coordinate correction
    # cc_y = 0.0  # 使用其它的坐标方案

    if ac_num == 1:
        new_obstacle_state.pose.position.x = -1.0  # 1.0
        new_obstacle_state.pose.position.y = -4.0
    elif ac_num == 2:
        new_obstacle_state.pose.position.x = -5.0  # 1.0
        new_obstacle_state.pose.position.y = 5.0
    elif ac_num == 3:
        new_obstacle_state.pose.position.x = 2.0  # 1.0
        new_obstacle_state.pose.position.y = -3.0
    elif ac_num == 4:
        new_obstacle_state.pose.position.x = 5.0  # 1.0
        new_obstacle_state.pose.position.y = 5.0
    else:
        gym.logger.warn("Error ac_num !")

    new_obstacle_state.pose.position.z = 0
    # 注意４元数的概念
    new_obstacle_state.pose.orientation.x = 0  # z与w 控制了水平朝向？
    new_obstacle_state.pose.orientation.y = 0
    new_obstacle_state.pose.orientation.z = 0
    new_obstacle_state.pose.orientation.w = 0

    new_obstacle_state.twist.linear.x = 0
    new_obstacle_state.twist.linear.y = 0
    new_obstacle_state.twist.linear.z = 0

    new_obstacle_state.twist.angular.x = 0
    new_obstacle_state.twist.angular.y = 0
    new_obstacle_state.twist.angular.z = 0

    new_obstacle_state.reference_frame = "world"

    return pub, new_obstacle_state


# def correct_coordinate_func():
#     if enable_correction:
#         def this_func(input_value):
#             return abs(input_value)
#     else:
#         def this_func(input_value):
#             return input_value
#     return this_func


# 没有0号robot
# 有障碍物的空间
# goal_sequence = [[(0.0, 0.0), (0.0, 0.0), (0.0, 0.0)],
#                  [(5.0, 5.0), (9.0, 5.0), (9.0, 1.0)]]

# goal_sequence = [[(0.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)],
#                  [(5.0, 5.0), (8.0, 7.0), (9.0, 1.0), (1.0, 8.9)]]
#
# initial_sequence = [[(0.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0, 0)],
#                     [(2.1, 1.2), (7.0, 3.0), (5.0, 8.2), (1.0, 6.2)]
#                     ]

# # 由于计算距离与角度,所以状态中正负不重要
# initial_sequence = [[(0.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)],
#                     [(1.5, 5.0), (9.0, 1.0), (5.0, 6.0), (2.0, 8.5)],
#                     [(2.0, -5.0), (8.5, -1.5), (5.0, -6.0), (2.0, -8.5)],
#                     [(-1.5, -5.0), (-9.0, -1.0), (-5.0, -6.0), (-2.0, -8.5)],
#                     [(-1.5, 5.0), (-9.0, 1.0), (-5.0, 6.0), (-2.0, 8.5)]]
#
# goal_sequence = [[(0.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)],
#                  [(8.0, 8.0), (2.0, 1.0), (6.0, 3.0), (9.0, 9.0)],
#                  [(8.0, -8.0), (2.0, -1.0), (6.0, -3.0), (9.0, -9.0)],
#                  [(-8.0, -8.0), (-2.0, -1.0), (-6.0, -3.0), (-9.0, -9.0)],
#                  [(-8.0, 8.0), (-2.0, 1.0), (-6.0, 3.0), (-9.0, 9.0)]]

# 由于计算距离与角度,所以状态中正负不重要
# for obstacle place, 多机器人障碍物 initial 并没有用到
initial_sequence = [[(0.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)],
                    [(1.5, 5.0), (9.0, 1.0), (5.0, 6.0), (2.0, 8.5)],
                    [(2.0, -5.0), (8.5, -1.5), (5.0, -6.0), (2.0, -8.5)],
                    [(-1.5, -5.0), (-9.0, -1.0), (-5.0, -6.0), (-2.0, -8.5)],
                    [(-1.5, 5.0), (-9.0, 1.0), (-5.0, 6.0), (-2.0, 8.5)]]

goal_sequence = [[(0.0, 0.0), (0.0, 0.0), (0.0, 0.0)],
                 [(-1.0, -8.0), (-9.0, -2.0), (-8.0, -8.0)],
                 [(-1.0, 5.0), (-9.0, 9.0), (-8.0, 2.0)],
                 [(5.0, -5.0), (9.0, -5.0), (2.0, -9.0)],
                 [(9.0, 3.0), (1.0, 7.0), (9.0, 9.0)]]


class SixthRobGpw(gym.Env):
    def __init__(self):
        rospy.init_node("rl_robot_node"+str(ac_num))
        self.pub, self.reset_robot_state = reset_node_init()
        self.act_pub = rospy.Publisher('/robot'+str(ac_num)+'/pioneer/cmd_vel', Twist, queue_size=1)
        self.sub_name = '/robot' + str(ac_num) + '/laser/scan'
        self.laser_sub = rospy.Subscriber(self.sub_name, LaserScan, self.laser_callback, queue_size=1)
        # self.cc_func = correct_coordinate_func()
        # self.observation_space = gym.spaces.Box(-10.0, 10.0, (15, ), dtype=np.dtype(np.float32))
        # if enable_correction:
        #     self.observation_space = gym.spaces.Box(0, 100, (14,), dtype=np.dtype(np.int32))  # 整数版
        # else:
        #     self.observation_space = gym.spaces.Box(-100, 100, (14, ), dtype=np.dtype(np.int32))  # 整数版
        # self.observation_space = gym.spaces.Box(-10.0, 10.0, (4, ), dtype=np.dtype(np.float32))
        self.observation_space = gym.spaces.Box(-10.0, 10.0, (17, ), dtype=np.dtype(np.float32))
        self.action_space = Discrete(4)  # 取值0,1,2,3,4,5　，　动作是否应该加上组合？
        self.state = None
        # self.laser_data = [0 for i in range(0, 10)]
        self.num_laser = 10  # 别乱改,有些相关数据没用这个参数
        self.laser_data = None
        self.sample_laser_data = [0 for _ in range(0, self.num_laser)]  # 特定抽取的data, 位置从右向左[0.0, 10.0]
        self.int_sample_laser_data = [0 for _ in range(0, self.num_laser)]  # 特定抽取的data, 位置从右向左,整数版
        # self.state_sample_laser_data = [0 for x in range(0, self.num_laser)]  # 特定抽取的data, 位置从右向左,归一化
        # 用于修正激光数据
        self.last_laser_data = [0 for _ in range(0, self.num_laser)]
        # self.current_sate = None
        # self.speed = 0.0  # 运行一段时间之后,记录的speed、angle就会与实际（gazebo）有较大差距
        # self.state_speed = 0.0  # 作为状态用的speed 离散整数化
        # self.state_angle = 0.0  # angular speed
        self.min_speed = 0.0  # -0.75  # 线速度 m/s ；角速度 rad/s（6弧度相当于一个整圆）,要想倒车必须在车身后部安装测距手段
        self.max_speed = 0.3  # 速度不能高于0.75,已经开始出现误差  0.5m/s因为环境障碍物比较密集
        # self.angle = 0.0
        self.min_angle = -0.2
        self.max_angle = +0.2
        self.cmd_twist = Twist()
        self.done = False
        # 获取状态服务
        rospy.wait_for_service('gazebo/get_model_state')
        self.get_state_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        # 重置位置服务
        rospy.wait_for_service('gazebo/set_model_state')
        self.set_obstacle_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        # 重置simulation
        rospy.wait_for_service('/gazebo/reset_simulation')
        self.reset_sim_srv = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        # self.obstacle_pose = [[0 for x in range(0, 2)] for x in range(0, 20)]  # 20个障碍物的位置[x, y]
        # gazebo返回的robot数据
        self.gazebo_robot_pose_x = 0.0  # 可以考虑为其注入噪声,这里在初始化时为0,导致下面一开始角度计算有一个错误
        self.gazebo_robot_pose_y = 0.0
        self.gazebo_robot_speed = 0.0
        self.gazebo_robot_angle = 0.0
        self.robot_orientation_z = 0
        self.robot_orientation_w = 0
        self.state_distance = 0.0  # 用于作为状态输入
        self.true_state_distance = 0.0
        self.state_polar_angle = 0.0  # 这个是robot相对x轴朝向
        self.last_robot_pose_x = 0.0
        self.last_robot_pose_y = 0.0
        # reward 相关
        self.arrive_reward = 100.0
        self.collision_reward = -100.0
        self.road_reward_times = 0.1  # 0.1
        self.time_cost_reward = 0  # -0.1
        self.last_distance_to_goal = 0.0  # 上一次动作时距离目标的距离
        self.distance_to_goal_threshold = 0.15  # 0.20
        self.collision_threshold = 0.34  # 至少比车身宽 0.5 , 0.43, 0.35是比较好的数值,再小就不行了
        # self.action_0 = False
        # self.distance_r_sate = 0
        # 设置最终robot位姿
        self.goal_x = 9.0  # 8
        self.goal_y = 6.0  # 8
        # self.desire_final_speed = None
        # self.desire_final_angle = None
        self.eps = 0  # 记录当前迭代轮数
        # self.get_obstacle_pose()
        self.seed()
        self.steps = 0  # for debug
        self.total_steps = 0
        self.trace_need = 1
        # self.reset()  # 先初始化一下 , 在dqn中会先调用一下的

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def laser_callback(self, msg):
        self.laser_data = msg.ranges

    # 使用随机目标应该可以加快学习速度,和泛化能力
    # def random_goal(self):
    #     # print("random_goal")
    #     # RandomState(ac_num)
    #     # 搞不懂random被openAI怎么动了,运行结果诡异,使用baselines中的uniform分布
    #     self.goal_x = self.np_random.uniform(low=-9.0, high=9.0)
    #     self.goal_y = self.np_random.uniform(low=-9.0, high=9.0)
    #     # self.goal_x = (18 * (np.random.ranf() - 0.5))  # x~(-9,9)
    #     # self.goal_y = (18 * (np.random.ranf() - 0.5))  # y~(-9,9)
    #     # print(str(self.goal_x) +"  " +str(self.goal_y))
    #     # 注意这里的设计, 每当goal 不满足条件时,就需要重新random一下,然后与出生点、“所有”障碍物点重新对比一下
    #     i = -1
    #     while i < 19:
    #         i += 1
    #         while self.compute_distance(self.goal_x, self.reset_robot_state.pose.position.x, self.goal_y,
    #                                     self.reset_robot_state.pose.position.y) < 1.0 \
    #                 or (self.compute_distance(self.goal_x, self.obstacle_pose[i][0],
    #                                           self.goal_y, self.obstacle_pose[i][1])) < 1.2:
    #             # self.goal_x = (18 * (np.random.ranf() - 0.5))  # 距离初始位置或任意障碍物过近
    #             # self.goal_y = (18 * (np.random.ranf() - 0.5))
    #             self.goal_x = self.np_random.uniform(low=-9.0, high=9.0)
    #             self.goal_y = self.np_random.uniform(low=-9.0, high=9.0)
    #             i = -1
    #
    #     self.goal_x = round(self.goal_x, 1)  # 离散化
    #     self.goal_y = round(self.goal_y, 1)
    #     # print("end random_goal")

    # fixed version
    def reset_goal(self):
        goal_num = self.np_random.randint(0, 3)
        # self.goal_x = self.cc_func(round(goal_sequence[ac_num][goal_num][0], 1))
        # self.goal_y = self.cc_func(round(goal_sequence[ac_num][goal_num][1], 1))
        self.goal_x = round(goal_sequence[ac_num][goal_num][0], 1)
        self.goal_y = round(goal_sequence[ac_num][goal_num][1], 1)

    def initial_pose(self):
        pose_num = self.np_random.randint(0, 4)
        # self.reset_robot_state.pose.position.x = self.cc_func(round(initial_sequence[ac_num][goal_num][0], 1))
        # self.reset_robot_state.pose.position.y = self.cc_func(round(initial_sequence[ac_num][goal_num][1], 1))
        self.reset_robot_state.pose.position.x = round(initial_sequence[ac_num][pose_num][0], 1)
        self.reset_robot_state.pose.position.y = round(initial_sequence[ac_num][pose_num][1], 1)
        # print('init_actor' + str(ac_num) + ' --> x: ' + str(self.reset_robot_state.pose.position.x)
        #       + ' y: ' + str(self.reset_robot_state.pose.position.y))

    @staticmethod
    def compute_distance(lhx, lhy, rhx, rhy):
        return math.sqrt((lhx - rhx) ** 2 + (lhy - rhy) ** 2)

    # goal 在robot当前位置为基准的角度
    def cpose_goal_angle(self):
        return math.atan2(self.goal_y - self.gazebo_robot_pose_y, self.goal_x - self.gazebo_robot_pose_x)  # (-pi,pi]

    # # 使用相邻两次移动计算robot朝向,移动距离很小的话（特别有噪声的情况下,怎么可能计算的准确？需要借助IMU?）
    # def compute_polar_coordinates(self):
    #     edge_a = self.compute_distance(self.last_robot_pose_x, self.last_robot_pose_y,
    #                                    self.gazebo_robot_pose_x, self.gazebo_robot_pose_y)
    #     edge_b = self.compute_distance(self.last_robot_pose_x, self.last_robot_pose_y, self.goal_x, self.goal_y)
    #     edge_c = self.compute_distance(self.gazebo_robot_pose_x, self.gazebo_robot_pose_y, self.goal_x, self.goal_y)
    #     cos_angle_beta = (edge_a**2 + edge_c**2 - edge_b**2)/(2*edge_a*edge_c)
    #     self.state_distance = edge_c  # 数值需进行处理
    #     self.state_angle = math.pi - math.acos(cos_angle_beta)  # 数值需进行处理

    # def distance_to_obstacle(self, num_x, num_y, count):
    #     return math.sqrt((self.obstacle_pose[count][0] - num_x) ** 2 + (self.obstacle_pose[count][1] - num_y) ** 2)

    # 如果使用normalization_state_input(),则不要使用这个
    # def state_speed_wrapper(self):
    #     # print(self.gazebo_robot_speed)
    #     # print(self.gazebo_robot_angle)
    #     self.state_speed = round(self.gazebo_robot_speed, 4) * 200
    #     self.state_speed = int(self.state_speed)  # (0~100)
    #     self.state_speed = np.clip(self.state_speed, 0, 100)
    #     self.state_angle = round(self.gazebo_robot_angle, 4) * 100
    #     self.state_angle = int(self.state_angle) + 50  # (0~100)
    #     self.state_angle = np.clip(self.state_angle, 0, 100)

    def reset(self):
        self.total_steps += self.steps
        self.steps = 0  # for debug
        self.eps += 1
        # print('call_reset')
        self.done = False
        # 每10轮random一次
        if self.eps % 10 == 0 or self.eps == 1:
            self.reset_goal()
            # self.initial_pose()
            # robot 的初始朝向也随机化
            self.reset_robot_state.pose.orientation.z = self.np_random.uniform(low=-0.9, high=0.9)
            self.reset_robot_state.pose.orientation.w = self.np_random.uniform(low=-0.9, high=0.9)
            print('actor'+str(ac_num)+' --> x: ' + str(self.goal_x) + ' y: ' + str(self.goal_y))
        # 重置robot的 speed angle 注意各步骤执行顺序
        self.cmd_twist.linear.x = 0.0
        self.cmd_twist.angular.z = 0.0
        # 将robot重置回原位置
        start_time = time.time()
        while (time.time() - start_time) < 0.25:  # 这里不使用srv直接重置,
            self.act_pub.publish(self.cmd_twist)  # 先重置速度
            self.pub.publish(self.reset_robot_state)
            time.sleep(0.05)
        # time.sleep(0.5)  # 重置后需要一定时间等待激光等数据更新...
        # 重置state中的pose speed angle
        # 每过大约10万步reset simulation
        if self.total_steps >= (self.trace_need*10000):
            # self.reset_sim_srv()
            self.trace_need += 1

        self.gazebo_robot_pose_x, self.gazebo_robot_pose_y, self.gazebo_robot_speed, self.gazebo_robot_angle,\
        self.state_polar_angle = self.get_gazebo_state()
        self.gazebo_robot_speed = 0.0
        self.gazebo_robot_angle = 0.0
        # self.laser_data = self.thread1.laser_data
        self.laser_sub = rospy.Subscriber(self.sub_name, LaserScan, self.laser_callback, queue_size=1)
        self.last_robot_pose_x = self.gazebo_robot_pose_x
        self.last_robot_pose_y = self.gazebo_robot_pose_y
        self.sample_fuc()
        self.get_rid_of_bad_data()
        self.true_state_distance = self.distance_to_goal()
        # self.state_distance = int(round(self.true_state_distance * 10))
        # if self.state_distance >= 10.0:
        #     self.state_distance = 10.0
        # self.normalization_state_input()
        # self.discretization_state_input()
        # print(self.sample_laser_data)
        # print(self.rew_sample_laser_data)
        # reset 函数需要提供初始状态
        # self.state = np.array([])  # 用来只有目标点作为状态时
        self.state = np.array(self.sample_laser_data[:])
        # self.state = np.append(self.state, [self.state_distance, self.state_polar_angle,
        #                                     self.gazebo_robot_speed, self.gazebo_robot_angle])
        self.state = np.append(self.state, [self.goal_x, self.goal_y,
                                            self.gazebo_robot_pose_x, self.gazebo_robot_pose_y, self.state_polar_angle,
                                            self.gazebo_robot_speed, self.gazebo_robot_angle])
        self.last_distance_to_goal = self.true_state_distance
        self.last_laser_data = self.sample_laser_data[:]
        # self.state = self.state.reshape((14,))
        self.state = self.state.reshape((17,))
        return self.state

    def step(self, action):
        self.steps += 1
        assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))
        # robot 的加速度是有限制的,这里的选取一定要慎重,有两种方案,要么把变化数值都设成很小(不宜,难以快速响应环境)
        # 要么将publish的频率设限,20hz(+0.1情况下)以下
        # print("action:"+str(action))
        # print("gazebo: speed" + str(self.gazebo_robot_speed))
        # action = 0  # testing
        if action == 0:
            self.gazebo_robot_speed += 0.05
        elif action == 1:
            self.gazebo_robot_speed -= 0.05
        # elif action == 2:
        #     self.gazebo_robot_speed = 0.0
        elif action == 2:
            self.gazebo_robot_angle += 0.05
        elif action == 3:
            self.gazebo_robot_angle -= 0.05
        # elif action == 5:
        #     self.gazebo_robot_angle = 0.0
        # elif action == 4:
        #     pass  # 保持当前状态
        else:
            gym.logger.warn("Unknown situation !")
        # if action == 0:
        #     self.gazebo_robot_speed = 0.10  # 速度不能突然设置为特别大的数,否则会失控,不能超过0.1了
        #     # self.action_0 = True
        # elif action == 1:
        #     self.gazebo_robot_speed = 0.0
        #     # self.action_0 = False
        # # elif action == 2:
        # #     self.gazebo_robot_speed = 0.0
        # elif action == 2:
        #     self.gazebo_robot_angle = +0.10
        #     # self.action_0 = False
        # elif action == 3:
        #     self.gazebo_robot_angle = -0.10
        #     # self.action_0 = False
        # # elif action == 5:
        # #     self.gazebo_robot_angle = 0.0
        # elif action == 4:
        #     self.gazebo_robot_angle = 0
        #     # self.action_0 = False
        # else:
        #     gym.logger.warn("Unknown situation !")

        self.gazebo_robot_speed = np.clip(self.gazebo_robot_speed, self.min_speed, self.max_speed)
        self.gazebo_robot_angle = np.clip(self.gazebo_robot_angle, self.min_angle, self.max_angle)

        # 与许多其它os不同之处,p2os设置好cmd_vel后,robot会一直保持该速度,
        # self.cmd_twist.linear.x = 0
        # gazebo中robot就算什么指令都没有,还是会有非常小的cmd_twist.linear.x与cmd_twist.angular.z,如果直接publish这些数据,只会造成误差越来越大
        if abs(self.gazebo_robot_speed) < 0.025:
            self.gazebo_robot_speed = 0.0
        if abs(self.gazebo_robot_angle) < 0.025:
            self.gazebo_robot_angle = 0.0
        self.cmd_twist.linear.x = self.gazebo_robot_speed
        self.cmd_twist.angular.z = self.gazebo_robot_angle
        # attention! testing!
        # self.cmd_twist.linear.x = 0.5
        # self.cmd_twist.angular.z = 0.0
        # print("gazebo: speed" + str(self.gazebo_robot_speed) + ",  angle: " + str(self.gazebo_robot_angle))
        # 应该新建一个服务,将最新的动作信息同步给该服务,在该服务中负责发布cmd_twist,或者使用线程
        time.sleep(0.075)  # 0.075
        self.act_pub.publish(self.cmd_twist)

        # 记录部分上一轮数据
        # self.laser_data = self.thread1.laser_data
        self.laser_sub = rospy.Subscriber(self.sub_name, LaserScan, self.laser_callback, queue_size=1)
        self.last_robot_pose_x = self.gazebo_robot_pose_x
        self.last_robot_pose_y = self.gazebo_robot_pose_y
        # 执行完动作,检查一下状态值(这里的严谨性有待考察,因为这里并不是严格的action一下,返回一个状态)
        self.gazebo_robot_pose_x, self.gazebo_robot_pose_y, self.gazebo_robot_speed, self.gazebo_robot_angle,\
        self.state_polar_angle = self.get_gazebo_state()
        self.sample_fuc()
        self.get_rid_of_bad_data()
        self.true_state_distance = self.distance_to_goal()
        # self.state_distance = int(round(self.true_state_distance * 10))
        # if self.state_distance >= 10:
        #     self.state_distance = 10
        # print(self.state_speed)
        # print(self.state_angle)
        # self.normalization_state_input()
        # self.discretization_state_input()
        # print(self.sample_laser_data)
        # print(self.rew_sample_laser_data)
        self.state = np.array(self.sample_laser_data[:])
        # self.state = np.array([])
        self.state = np.append(self.state, [self.goal_x, self.goal_y,
                                            self.gazebo_robot_pose_x, self.gazebo_robot_pose_y, self.state_polar_angle,
                                            self.gazebo_robot_speed, self.gazebo_robot_angle])
        # self.state = np.append(self.state, [self.state_distance, self.state_polar_angle,
        #                                     self.gazebo_robot_speed, self.gazebo_robot_angle])
        # print(self.sample_laser_data)
        # 应该把最后的位姿信息也作为限制, min(distance) 选择0.25,有待考察,因为将测距仪放在车头前部
        if self.true_state_distance < self.distance_to_goal_threshold \
                or min(self.sample_laser_data) < self.collision_threshold:  # and (self.gazebo_robot_speed < 0.1):
            self.done = True
            # print("SPACE")
            # print(self.sample_laser_data)
            # print(self.last_laser_data)
            # print("x: " + str(self.gazebo_robot_pose_x) + "y: " + str(self.gazebo_robot_pose_y))
            # print(self.true_state_distance)
            if self.steps <= 10:  # for debug
                print("robot: "+str(ac_num)+" : ")
                print(self.sample_laser_data)
                # print(self.last_laser_data)
                print("goal_x: "+str(self.goal_x)+"goal_y: "+str(self.goal_y))
                print("init_x: "+str(self.gazebo_robot_pose_x)+"init_y: "+str(self.gazebo_robot_pose_y))
                # print(self.int_sample_laser_data)
        self.state = self.state.reshape((17, ))
        reward_num = self.reward()  # reward值必须在更新last_distance_to_goal之前计算
        self.last_distance_to_goal = self.true_state_distance  # 更新距离,为下次计算reward准备
        self.last_laser_data = self.sample_laser_data[:]  # 注意这里,坑....  copy!!!

        # if ac_num == 1:
        #     print("actor"+str(ac_num)+": ")
        #     print(self.state)
        #     print(reward_num)
        return self.state, reward_num, self.done, {}

    def distance_to_goal(self):
        return math.sqrt((self.goal_x - self.gazebo_robot_pose_x)**2 + (self.goal_y - self.gazebo_robot_pose_y)**2)

    def reward(self):
        # 应该把最后的位姿信息也作为reward的限制,与done保持一致,目标距离与障碍物距离之间应该有交叉reward?
        if self.true_state_distance < self.distance_to_goal_threshold:
            return self.arrive_reward
        elif min(self.sample_laser_data) < self.collision_threshold:  # 有关障碍物距离部分,原始激光数据有噪声（或其它原因,不能直接做判定,甚至达到0.2 ???）
            # print(self.sample_laser_data)
            # print("error")
            return self.collision_reward
        # 谨慎选择有关距离的reward_fuc, 避免robot有可能出现的刷分行为(与arrive_reward的比例,正负等等)
        # 有关距离目标距离的部分,应该可以加快学习速度,但是比较难处理,超过目标点之后,跑远了就给它负的reward
        # 方案一
        # else:
        #     return self.road_reward_times * (self.last_distance_ro_goal-self.distance_to_goal())
        # 方案二
        else:  # 应该设置成总选action_1给负分
            return (self.last_distance_to_goal - self.true_state_distance) * 10
            # if self.action_0 is True:
            #     plus_reward = 0.1
            # else:
            #     plus_reward = 0
            # if self.last_distance_to_goal - self.state_distance > 0:  # 越来越近
            #     return 1.0 * self.road_reward_times + plus_reward
            # elif self.last_distance_to_goal - self.state_distance < 0:  # 远离惩罚
            #     return -0.5 * self.road_reward_times + plus_reward  # 可以加大远离的惩罚
            # else:
            #     return 0 + plus_reward
            # 连续状态下,这里应该不用0作为区分标准,maybe 0.5 ?
            # return (self.last_distance_to_goal - self.state_distance) * self.road_reward_times
            # if self.last_distance_to_goal - self.state_distance > 0:  # 越来越近
            #     return 1.0*self.road_reward_times + self.time_cost_reward
            # elif self.last_distance_to_goal - self.state_distance <= 0:  # 远离惩罚,不动惩罚
            #     return -0.5*self.road_reward_times + self.time_cost_reward   # 可以加大远离的惩罚
            # else:
            #     return 0 + self.time_cost_reward
        # else:
        #     return 0.0

    def render(self, mode='human'):
        pass

    # @staticmethod
    # def normalization_func(max_num, min_num, x):
    #     return (x - min_num)/(max_num - min_num)
    #
    # def normalization_state_input(self):
    #     for i in range(0, 10):
    #         self.state_sample_laser_data[i] = self.normalization_func(10, 0, self.sample_laser_data[i])
    #     self.state_polar_angle = self.normalization_func(2*math.pi, 0, self.state_polar_angle)
    #     self.state_distance = self.normalization_func(14.14, 0, self.true_state_distance)  # sqrt(200)
    #     self.state_speed = self.normalization_func(0.5, 0, self.gazebo_robot_speed)
    #     if self.state_speed < 0:  # why?
    #         self.state_speed = 0.0
    #     self.state_angle = self.normalization_func(0.5, -0.5, self.gazebo_robot_angle)
    #
    # # round()取值方式精度不高,具体查看 http://www.runoob.com/w3cnote/python-round-func-note.html
    # def discretization_state_input(self):
    #     for i in range(0, 10):
    #         self.state_sample_laser_data[i] = round(self.state_sample_laser_data[i], 2)
    #     self.state_polar_angle = round(self.state_polar_angle, 2)
    #     self.state_distance = round(self.state_distance, 2)
    #     self.state_speed = round(self.state_speed, 2)
    #     self.state_angle = round(self.state_angle, 2)

    # 是否应该换成机器人本身的数据,毕竟真实机器人没有这些
    def get_gazebo_state(self):
        # 第一个参数model_name,第二个参数为relative_entity_name
        model_state = self.get_state_srv('robot'+str(ac_num), '')  # 注意model_state 的类型为GetModelStateResponse
        robot_pose_x = round(model_state.pose.position.x, 1)  # 注意pose为(x,y,z) , 离散,
        robot_pose_y = round(model_state.pose.position.y, 1)
        robot_twist_linear_x = round(model_state.twist.linear.x, 4)  # 离散化
        robot_twist_angular_z = round(model_state.twist.angular.z, 4)  # angular.z代表平面机器人的角速度，因为此时z轴为旋转轴
        if model_state.pose.orientation.z < 0:  # robot_polar_angle位于[-2pi,2pi],what the hell?
            robot_polar_angle = -2 * math.acos(model_state.pose.orientation.w)
        else:
            robot_polar_angle = 2 * math.acos(model_state.pose.orientation.w)
        # print(robot_polar_angle)
        # print(model_state.pose.orientation.w)
        # if robot_polar_angle < 0:  # 整合为[0,2pi]
        #     robot_polar_angle += 2*math.pi  # robot_polar_angle是robot相对默认坐标系的转向角
        if robot_polar_angle > math.pi:  # 整合为(-pi,pi]
            robot_polar_angle -= 2*math.pi  # robot_polar_angle是robot相对默认坐标系的转向角
        elif robot_polar_angle <= -math.pi:
            robot_polar_angle += 2*math.pi
        # print(robot_polar_angle/math.pi * 180)
        # robot_to_goal_angle = self.cpose_goal_angle() - robot_polar_angle  # 这里会被计算成（-2pi,2pi]
        # if robot_to_goal_angle > math.pi:  # 整合为(-pi,pi]
        #     robot_to_goal_angle -= 2*math.pi  # robot_polar_angle是robot相对默认坐标系的转向角
        # elif robot_to_goal_angle <= -math.pi:
        #     robot_to_goal_angle += 2*math.pi
        # print("robot 当前与坐标轴的角度")
        # print(robot_polar_angle / math.pi * 180)
        # print("goal 与当前robot坐标轴的角度")
        # print(self.cpose_goal_angle() / math.pi * 180)
        # print("goal 与当前robot的角度")
        # print(robot_to_goal_angle/math.pi * 180)
        # robot_to_goal_angle += math.pi  # 变为(0,2pi]
        # robot_to_goal_angle = robot_to_goal_angle*10  # normalization 注释掉这句话
        # robot_orientation_z = round(50*(model_state.pose.orientation.z+1), 1)  # z与w 在[-1,1]之间
        # robot_orientation_w = round(50*(model_state.pose.orientation.w+1), 1)
        # return self.cc_func(robot_pose_x), self.cc_func(robot_pose_y), robot_twist_linear_x, robot_twist_angular_z
        # return robot_pose_x, robot_pose_y, robot_twist_linear_x, robot_twist_angular_z, robot_to_goal_angle
        return robot_pose_x, robot_pose_y, robot_twist_linear_x, robot_twist_angular_z, robot_polar_angle

    # 当前实现使用其它方法
    # 获取障碍物的位置
    # def get_obstacle_pose(self):
    #     # print('call get_obstacle_pose')
    #     for i in range(0, 15):  # 15个方块
    #         obstacle_pose = self.get_state_srv('unit_box_' + str(i), '')
    #         self.obstacle_pose[i][0] = obstacle_pose.pose.position.x
    #         self.obstacle_pose[i][1] = obstacle_pose.pose.position.y
    #
    #     for i in range(0, 5):  # 5个圆柱体,不使用球体,容易到处乱滚
    #         obstacle_pose = self.get_state_srv('unit_cylinder_' + str(i), '')
    #         self.obstacle_pose[i + 15][0] = obstacle_pose.pose.position.x
    #         self.obstacle_pose[i + 15][1] = obstacle_pose.pose.position.y
    #     # print('end call get_obstacle_pose')
    #
    # # 用来随机重置障碍物的位置坐标
    # def reset_obstacle_pose(self):
    #     for i in range(0, 20):
    #         self.obstacle_pose[i][0] = (20 * (np.random.ranf() - 0.5))  # x~(-10,10)
    #         while abs(self.obstacle_pose[i][0]) < 1.0:
    #             self.obstacle_pose[i][0] = (20 * (np.random.ranf() - 0.5))  # 距离初始位置过近
    #         self.obstacle_pose[i][0] = round(self.obstacle_pose[i][0], 1)  # 离散化
    #         self.obstacle_pose[i][1] = (20 * (np.random.ranf() - 0.5))  # y~(-10,10)
    #         while abs(self.obstacle_pose[i][1]) < 1.0:
    #             self.obstacle_pose[i][1] = (20 * (np.random.ranf() - 0.5))  # 距离初始位置过近
    #         self.obstacle_pose[i][1] = round(self.obstacle_pose[i][1], 1)
    #
    # # 使用服务,重置障碍物
    # def reset_obstacle_pose_srv(self):
    #     new_obstacle_state = ModelState()
    #
    #     new_obstacle_state.pose.orientation.x = 0
    #     new_obstacle_state.pose.orientation.y = 0
    #     new_obstacle_state.pose.orientation.z = 0
    #     new_obstacle_state.pose.orientation.w = 0
    #
    #     new_obstacle_state.twist.linear.x = 0
    #     new_obstacle_state.twist.linear.y = 0
    #     new_obstacle_state.twist.linear.z = 0
    #
    #     new_obstacle_state.twist.angular.x = 0
    #     new_obstacle_state.twist.angular.y = 0
    #     new_obstacle_state.twist.angular.z = 0
    #
    #     new_obstacle_state.reference_frame = "world"
    #     for i in range(0, 15):  # 10个方块
    #         new_obstacle_state.model_name = "unit_box_"+str(i)
    #         new_obstacle_state.pose.position.x = self.obstacle_pose[i][0]
    #         new_obstacle_state.pose.position.y = self.obstacle_pose[i][1]
    #         new_obstacle_state.pose.position.z = 0
    #
    #         self.set_obstacle_srv(new_obstacle_state)
    #
    #     for i in range(0, 5):  # 5个圆柱体,不使用球体,容易到处乱滚
    #         new_obstacle_state.model_name = "unit_cylinder_"+str(i)
    #         new_obstacle_state.pose.position.x = self.obstacle_pose[i+15][0]
    #         new_obstacle_state.pose.position.y = self.obstacle_pose[i+15][1]
    #         new_obstacle_state.pose.position.z = 0
    #
    #         self.set_obstacle_srv(new_obstacle_state)

    # 从原激光数据中抽取10组0.0型的激光数据,相当于每20°取一个;
    # 为了各robot可以互相探测,激光安装在车身前部,但更合理的做法应当是车身中央,所以要将测量到的laser数据进行一下变换0.15
    def sample_fuc(self):
        self.sample_laser_data[0] = self.laser_data[0]
        self.sample_laser_data[-1] = self.laser_data[-1]
        for i in range(1, 9):
            self.sample_laser_data[i] = self.laser_data[i * 72]
        for j in range(0, 10):
            # temp_angle = (math.pi / 9) * j  # 弧度制
            # self.sample_laser_data[j] += math.sin(temp_angle) * 0.15  # 修正中心位置,这个应该用于安装于车头的情况
            if self.sample_laser_data[j] > 10.0:
                self.sample_laser_data[j] = 10.0

    # # 该函数应该在sample_fuc()之后调用,用于记录一个数组中的几组激光数据,self.rew_sample_laser_data为3×10（暂时）
    # def add_check_laser_data(self):
    #     data = self.sample_laser_data
    #     print(data)
    #     if self.next_nrsld_index >= len(self.rew_sample_laser_data):
    #         self.rew_sample_laser_data.append(data)
    #     else:
    #         self.rew_sample_laser_data[self.next_nrsld_index] = data
    #     self.next_nrsld_index = (self.next_nrsld_index + 1) % self.num_rew_sample_laser_data
    #     print(self.next_nrsld_index)
    # def add_check_laser_data(self):
    #     data = self.sample_laser_data
    #     self.rew_sample_laser_data[self.next_nrsld_index] = data
    #     self.next_nrsld_index = (self.next_nrsld_index + 1) % self.num_rew_sample_laser_data
    #     print("what the hell?")
    #     print(self.rew_sample_laser_data)
    #     print(self.next_nrsld_index)
    #
    # # 用于恢复<0.3的激光数据
    # def get_rid_of_bad_data(self):
    #     for i in range(self.num_laser):
    #         if self.sample_laser_data[i] < 0.3:
    #             for j in range(self.num_rew_sample_laser_data):
    #                 if self.rew_sample_laser_data[j][i] > 0.3:
    #                     self.sample_laser_data[i] = self.rew_sample_laser_data[j][i]
    #                 else:
    #                     pass
    #         else:
    #             pass

    # 用于恢复<0.3的激光数据,或许应该检查的是相同位置激光读数差距(应该不只保存一组,而是多组，方便对比)
    def get_rid_of_bad_data(self):
        for i in range(self.num_laser):
            if self.sample_laser_data[i] < 0.3:
                # print("check")
                # print(self.sample_laser_data)
                # print(self.last_laser_data)
                self.sample_laser_data[i] = self.last_laser_data[i]
            else:
                pass

    # 如果使用normalization_state_input(),则不要使用这个
    def int_sample_fuc(self):
        for j in range(0, 10):
            self.int_sample_laser_data[j] = int(self.sample_laser_data[j]*10)  # 变成整数

    # 找到其它原因,放弃该部分代码
    # # 给出一个避免噪声影响的最小值, 还要加上限制条件（inf情况下）,如安然变小（噪声）,为false
    # def check_min_laser_data(self):
    #     mean = 0.0
    #     sigle_bool_check_data = [0 for x in range(0, self.num_rew_sample_laser_data)]
    #     sigle_bool_bool = [0 for x in range(0, self.num_rew_sample_laser_data)]
    #     for i in range(self.num_laser):
    #         for j in range(self.num_rew_sample_laser_data):
    #             sigle_bool_check_data[j] = self.rew_sample_laser_data[j][i]
    #             mean = sum(sigle_bool_check_data)/len(sigle_bool_check_data)
    #         for k in range(self.num_rew_sample_laser_data):
    #             if (sigle_bool_check_data[k] - mean) > 0.1:   # (当前状态下不太可能单次变化超过0.1)
    #                 sigle_bool_bool[k] = False  # 判定为噪声数据
    #             else:
    #                 sigle_bool_bool[k] = True
    #         if False in sigle_bool_bool:
    #             self.bool_check_laser_data[i] = False  # 判定该位置为噪声数据
    #     # 迭代寻找最小数据,未完成

    # def __del__(self):
    #     # 等待至线程中止。这阻塞调用线程直至线程的join()方法被调用中止-正常退出或者抛出未处理的异常-或者是可选的超时发生。
    #     self.thread1.join()  # 还是不对
    #
    def close(self):
        pass
