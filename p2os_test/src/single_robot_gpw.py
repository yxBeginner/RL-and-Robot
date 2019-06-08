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

enable_correction = True
ac_num = 1


def set_gpw_num(num):
    global ac_num
    ac_num = num

# 2.将reward进行分段设置
# 3.修改action作用方式,(1)不能speed为持久状态,(2)使用某种移动作为action

def reset_node_init():
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

    new_obstacle_state = ModelState()
    new_obstacle_state.model_name = "robot"+str(ac_num)

    if ac_num == 1:
        new_obstacle_state.pose.position.x = 2.0  # 1.0
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


# for obstacle place
initial_sequence = [[(0.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)],
                 [(1.5, 5.0), (9.0, 1.0), (5.0, 6.0), (2.0, 8.5)]]

goal_sequence = [[(0.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0, 0)],
                    [(8.0, 8.0), (2.0, 1.0), (6.0, 3.0), (9.0, 9.0)]]


class SingleRobGpw(gym.Env):
    def __init__(self):
        rospy.init_node("rl_robot_node"+str(ac_num))
        self.pub, self.reset_robot_state = reset_node_init()
        self.act_pub = rospy.Publisher('/robot'+str(ac_num)+'/pioneer/cmd_vel', Twist, queue_size=1)
        self.sub_name = '/robot' + str(ac_num) + '/laser/scan'
        self.laser_sub = rospy.Subscriber(self.sub_name, LaserScan, self.laser_callback, queue_size=1)
        self.observation_space = gym.spaces.Box(0, 100, (12, ), dtype=np.dtype(np.int32))
        self.action_space = Discrete(5)  # 取值0,1,2,3,4,5　，　动作是否应该加上组合？
        self.state = None
        self.num_laser = 10  # 别乱改,有些相关数据没用这个参数
        self.laser_data = None
        self.sample_laser_data = [0 for x in range(0, self.num_laser)]  # 特定抽取的data, 位置从右向左
        self.int_sample_laser_data = [0 for x in range(0, self.num_laser)]  # 特定抽取的data, 位置从右向左,整数版
        # 用于修正激光数据
        self.last_laser_data = [0 for x in range(0, self.num_laser)]
        self.state_speed = 0.0  # 作为状态用的speed 离散整数化
        self.state_angle = 0.0  # angular speed
        self.min_speed = 0.0  # -0.75  # 线速度 m/s ；角速度 rad/s（6弧度相当于一个整圆）,要想倒车必须在车身后部安装测距手段
        self.max_speed = 0.5  # 速度不能高于0.75,已经开始出现误差  0.5m/s因为环境障碍物比较密集
        self.min_angle = - 0.5
        self.max_angle = 0.5
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
        # gazebo返回的robot数据
        self.gazebo_robot_pose_x = 1.5  # 可以考虑为其注入噪声,这里在初始化时为0,导致下面一开始角度计算有一个错误
        self.gazebo_robot_pose_y = 5.0
        self.gazebo_robot_speed = 0.0
        self.gazebo_robot_angle = 0.0
        self.robot_orientation_z = 0
        self.robot_orientation_w = 0
        self.true_state_distance = 0.0  # self.state_distance 经过处理,不再是真实距离,用于检查是否到达目标
        self.state_distance = 0.0  # 用于作为状态输入,对比每步的reward
        self.state_angle = 0.0
        self.state_polar_angle = 0.0  # 这个是robot相对x轴朝向
        self.last_robot_pose_x = 0.0
        self.last_robot_pose_y = 0.0
        # reward 相关
        self.arrive_reward = 10.0
        self.collision_reward = -10.0
        self.road_reward_times = 0.1  # 0.1
        self.last_distance_to_goal = 0.0  # 上一次动作时距离目标的距离
        self.distance_to_goal_threshold = 0.20  # 0.25
        self.collision_threshold = 0.35  # 至少比车身宽 0.5 , 0.43, 0.35是比较好的数值,再小就不行了
        # self.action_0 = False
        # self.distance_r_sate = 0
        # 设置最终robot位姿
        self.goal_x = 9.0  # 8
        self.goal_y = 6.0  # 8
        self.eps = 0  # 记录当前迭代轮数
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

    # fixed version
    def reset_goal(self):
        goal_num = self.np_random.randint(0, 4)
        self.goal_x = round(goal_sequence[ac_num][goal_num][0], 1)
        self.goal_y = round(goal_sequence[ac_num][goal_num][1], 1)

    def initial_pose(self):
        pose_num = self.np_random.randint(0, 4)
        self.reset_robot_state.pose.position.x = round(initial_sequence[ac_num][pose_num][0], 1)
        self.reset_robot_state.pose.position.y = round(initial_sequence[ac_num][pose_num][1], 1)

    @staticmethod
    def compute_distance(lhx, lhy, rhx, rhy):
        return math.sqrt((lhx - rhx) ** 2 + (lhy - rhy) ** 2)

    # goal 在robot当前位置为基准的角度
    def cpose_goal_angle(self):
        return math.atan2(self.goal_y - self.gazebo_robot_pose_y, self.goal_x - self.gazebo_robot_pose_x)  # (-pi,pi]

    # 如果使用normalization_state_input(),则不要使用这个
    def state_speed_wrapper(self):
        self.state_speed = round(self.gazebo_robot_speed, 4) * 200
        self.state_speed = int(self.state_speed)  # (0~100)
        self.state_speed = np.clip(self.state_speed, 0, 100)
        self.state_angle = round(self.gazebo_robot_angle, 4) * 100
        self.state_angle = int(self.state_angle) + 50  # (0~100)
        self.state_angle = np.clip(self.state_angle, 0, 100)

    def reset(self):
        self.total_steps += self.steps
        self.steps = 0  # for debug
        self.eps += 1
        self.done = False
        # 每20轮random一次
        if self.eps % 20 == 0 or self.eps == 1:
            self.reset_goal()
            self.initial_pose()
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
        # 重置state中的pose speed angle
        # 每过大约10万步reset simulation
        if self.total_steps >= (self.trace_need*10000):
            self.trace_need += 1

        self.gazebo_robot_pose_x, self.gazebo_robot_pose_y, self.gazebo_robot_speed, self.gazebo_robot_angle,\
        self.state_polar_angle = self.get_gazebo_state()
        self.gazebo_robot_speed = 0.0
        self.gazebo_robot_angle = 0.0
        self.laser_sub = rospy.Subscriber(self.sub_name, LaserScan, self.laser_callback, queue_size=1)
        self.last_robot_pose_x = self.gazebo_robot_pose_x
        self.last_robot_pose_y = self.gazebo_robot_pose_y
        self.sample_fuc()
        self.get_rid_of_bad_data()
        self.int_sample_fuc()
        # reset 函数需要提供初始状态
        self.state_speed_wrapper()
        self.true_state_distance = self.distance_to_goal()
        self.state = np.array(self.int_sample_laser_data[:])
        self.state_distance = int(round(self.true_state_distance*6))
        self.state = np.append(self.state, [self.state_distance, int(self.state_polar_angle)])
        self.last_distance_to_goal = self.state_distance
        self.last_laser_data = self.sample_laser_data[:]
        self.state = self.state.reshape((12,))
        return self.state

    def step(self, action):
        self.steps += 1
        assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))
        if action == 0:
            self.gazebo_robot_speed = 0.10  # 速度不能突然设置为特别大的数,否则会失控,不能超过0.1了
            # self.action_0 = True
        elif action == 1:
            self.gazebo_robot_speed = 0.0
            # self.action_0 = False
        elif action == 2:
            self.gazebo_robot_angle = +0.10
            # self.action_0 = False
        elif action == 3:
            self.gazebo_robot_angle = -0.10
            # self.action_0 = False
        elif action == 4:
            self.gazebo_robot_angle = 0
            # self.action_0 = False
        else:
            gym.logger.warn("Unknown situation !")

        self.gazebo_robot_speed = np.clip(self.gazebo_robot_speed, self.min_speed, self.max_speed)
        self.gazebo_robot_angle = np.clip(self.gazebo_robot_angle, self.min_angle, self.max_angle)

        if abs(self.gazebo_robot_speed) < 0.025:
            self.gazebo_robot_speed = 0.0
        if abs(self.gazebo_robot_angle) < 0.025:
            self.gazebo_robot_angle = 0.0
        self.cmd_twist.linear.x = self.gazebo_robot_speed
        self.cmd_twist.angular.z = self.gazebo_robot_angle
        time.sleep(0.085)  # 0.075
        self.act_pub.publish(self.cmd_twist)

        # 记录部分上一轮数据
        self.laser_sub = rospy.Subscriber(self.sub_name, LaserScan, self.laser_callback, queue_size=1)
        self.last_robot_pose_x = self.gazebo_robot_pose_x
        self.last_robot_pose_y = self.gazebo_robot_pose_y
        # 执行完动作,检查一下状态值(这里的严谨性有待考察,因为这里并不是严格的action一下,返回一个状态)
        self.gazebo_robot_pose_x, self.gazebo_robot_pose_y, self.gazebo_robot_speed, self.gazebo_robot_angle,\
        self.state_polar_angle = self.get_gazebo_state()
        self.sample_fuc()
        self.get_rid_of_bad_data()
        self.true_state_distance = self.distance_to_goal()
        self.int_sample_fuc()
        self.state = np.array(self.int_sample_laser_data[:])
        self.state_speed_wrapper()
        self.state_distance = int(round(self.true_state_distance * 6))
        self.state = np.append(self.state, [self.state_distance, int(self.state_polar_angle)])
        # 应该把最后的位姿信息也作为限制, min(distance) 选择0.25,有待考察,因为将测距仪放在车头前部
        if self.true_state_distance < self.distance_to_goal_threshold \
                or min(self.sample_laser_data) < self.collision_threshold:  # and (self.gazebo_robot_speed < 0.1):
            self.done = True
            if self.steps <= 10:  # for debug
                print("robot: "+str(ac_num)+" : ")
                print(self.sample_laser_data)
                print("goal_x: "+str(self.goal_x)+"goal_y: "+str(self.goal_y))
                print("init_x: "+str(self.gazebo_robot_pose_x)+"init_y: "+str(self.gazebo_robot_pose_y))
        self.state = self.state.reshape((12, ))
        reward_num = self.reward()  # reward值必须在更新last_distance_to_goal之前计算
        self.last_distance_to_goal = self.state_distance  # 更新距离,为下次计算reward准备
        self.last_laser_data = self.sample_laser_data[:]  # 注意这里,坑....  copy!!!

        return self.state, reward_num, self.done, {}

    def distance_to_goal(self):
        return math.sqrt((self.goal_x - self.gazebo_robot_pose_x)**2 + (self.goal_y - self.gazebo_robot_pose_y)**2)

    def reward(self):
        # 应该把最后的位姿信息也作为reward的限制,与done保持一致,目标距离与障碍物距离之间应该有交叉reward?
        if self.true_state_distance < self.distance_to_goal_threshold:
            return self.arrive_reward
        elif min(self.sample_laser_data) < self.collision_threshold:  # 有关障碍物距离部分,原始激光数据有噪声（或其它原因,不能直接做判定,甚至达到0.2 ???）
            return self.collision_reward
        else:  # 应该设置成总选action_1给负分
            if self.last_distance_to_goal - self.distance_to_goal() > 0:  # 越来越近
                return 1.0*self.road_reward_times
            elif self.last_distance_to_goal - self.distance_to_goal() < 0:  # 远离惩罚,不动惩罚
                return -0.5*self.road_reward_times  # 可以加大远离的惩罚
            else:
                return 0
        # else:
        #     return 0.0

    def render(self, mode='human'):
        pass

    # 是否应该换成机器人本身的数据,毕竟真实机器人没有这些
    # @staticmethod
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
        if robot_polar_angle > math.pi:  # 整合为(-pi,pi]
            robot_polar_angle -= 2*math.pi  # robot_polar_angle是robot相对默认坐标系的转向角
        elif robot_polar_angle <= -math.pi:
            robot_polar_angle += 2*math.pi
        robot_to_goal_angle = self.cpose_goal_angle() - robot_polar_angle  # 这里会被计算成（-2pi,2pi]
        if robot_to_goal_angle > math.pi:  # 整合为(-pi,pi]
            robot_to_goal_angle -= 2*math.pi  # robot_polar_angle是robot相对默认坐标系的转向角
        elif robot_to_goal_angle <= -math.pi:
            robot_to_goal_angle += 2*math.pi
        robot_to_goal_angle += math.pi  # 变为(0,2pi]
        robot_to_goal_angle = round(robot_to_goal_angle*10)  # normalization 注释掉这句话
        return robot_pose_x, robot_pose_y, robot_twist_linear_x, robot_twist_angular_z, robot_to_goal_angle

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
            self.sample_laser_data[j] = round(self.sample_laser_data[j], 1)  # 离散化,保留1位小数,注意round()既非截断,也非四舍五入,也许2位更好？
            # if self.sample_laser_data[i] == float('inf'):
            if self.sample_laser_data[j] > 10.0:
                self.sample_laser_data[j] = 10.0

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

    def close(self):
        pass
        # self.thread1.join()  # 还是不对
