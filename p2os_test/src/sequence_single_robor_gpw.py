#!/usr/bin/python3
import time
import threading
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
# from gazebo_msgs.srv import GetModelStateResponse
# from rosgraph_msgs.msg import Clock

enable_correction = True
ac_num = 1


def set_gpw_num(num):
    global ac_num
    ac_num = num

# 使用连续4次状态作为输入
# 效果并不好,可能是设计上的问题
class LaserNodeThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.laser_data = None
        self.laser_sub = None
        self.sub_name = '/robot'+str(ac_num)+'/laser/scan'
        # self.count = 0

    def laser_callback(self, msg):
        self.laser_data = msg.ranges

    def run(self):
        while True:
            # self.count += 1
            self.laser_sub = rospy.Subscriber(self.sub_name,
                                              LaserScan, self.laser_callback, queue_size=1)  # 10
            time.sleep(0.05)  # 0.1
        # rospy.spin()


# 由于先前的写法,这里暂时只能单独写,而不能统一处理
def reset_node_init():
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

    new_obstacle_state = ModelState()
    new_obstacle_state.model_name = "robot"+str(ac_num)
    # cc_x = 0.0  # coordinate correction
    # cc_y = 0.0  # 使用其它的坐标方案

    if ac_num == 1:
        new_obstacle_state.pose.position.x = 1.0
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
goal_sequence = [[(0.0, 0.0), (0.0, 0.0), (0.0, 0.0)],
                 [(5.0, 5.0), (9.0, 5.0), (9.0, 1.0)]]


class SSingleRobGpw(gym.Env):
    def __init__(self):
        rospy.init_node("rl_robot_node"+str(ac_num))
        self.pub, self.reset_robot_state = reset_node_init()
        self.act_pub = rospy.Publisher('/robot'+str(ac_num)+'/pioneer/cmd_vel', Twist, queue_size=1)
        # 当前每步为16个状态
        if enable_correction:
            self.observation_space = gym.spaces.Box(0, 100, (64,), dtype=np.dtype(np.int32))  # 整数版
        else:
            self.observation_space = gym.spaces.Box(-100, 100, (64, ), dtype=np.dtype(np.int32))  # 整数版
        # self.observation_space = gym.spaces.Box(-10.0, 10.0, (4, ), dtype=np.dtype(np.float32))
        self.action_space = Discrete(6)  # 取值0,1,2,3,4,5　，　动作是否应该加上组合？
        self.state = None
        # self.laser_data = [0 for i in range(0, 10)]
        self.num_laser = 10  # 别乱改,有些相关数据没用这个参数
        self.laser_data = None
        self.sample_laser_data = [0 for x in range(0, self.num_laser)]  # 特定抽取的data, 位置从右向左
        self.int_sample_laser_data = [0 for x in range(0, self.num_laser)]  # 特定抽取的data, 位置从右向左,整数版
        # 用于修正激光数据
        # self.last_laser_data = [0 for x in range(0, self.num_laser)]
        # self.current_sate = None
        # self.speed = 0.0  # 运行一段时间之后,记录的speed、angle就会与实际（gazebo）有较大差距
        self.state_speed = 0.0  # 作为状态用的speed 离散整数化
        self.state_angle = 0.0  # angular speed
        self.s_state = None
        self.min_speed = 0.0  # -0.75  # 线速度 m/s ；角速度 rad/s（6弧度相当于一个整圆）,要想倒车必须在车身后部安装测距手段
        self.max_speed = 0.5  # 速度不能高于0.75,已经开始出现误差  0.5m/s因为环境障碍物比较密集
        # self.angle = 0.0
        self.min_angle = - 0.5
        self.max_angle = 0.5
        self.cmd_twist = Twist()
        self.done = False
        self.thread1 = LaserNodeThread()
        self.thread1.setDaemon(True)
        self.thread1.start()
        # 获取位置服务
        rospy.wait_for_service('gazebo/get_model_state')
        self.get_state_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        # 重置位置服务
        rospy.wait_for_service('gazebo/set_model_state')
        self.set_obstacle_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        # self.obstacle_pose = [[0 for x in range(0, 2)] for x in range(0, 20)]  # 20个障碍物的位置[x, y]
        # gazebo返回的robot数据
        self.gazebo_robot_pose_x = None  # 可以考虑为其注入噪声
        self.gazebo_robot_pose_y = None
        self.gazebo_robot_speed = 0.0
        self.gazebo_robot_angle = 0.0
        self.state_distance = 0.0
        self.state_angle = 0.0
        self.last_robot_pose_x = 0.0
        self.last_robot_pose_y = 0.0
        # reward 相关
        self.arrive_reward = 100.0
        self.collision_reward = -100.0
        self.road_reward_times = 0.1
        self.last_distance_to_goal = 0.0  # 上一次动作时距离目标的距离
        self.distance_to_goal_threshold = 0.25
        self.collision_threshold = 0.43  # 至少比车身宽 0.5  如果laser位于中央,0.3不足以做到覆盖车身距离
        self.distance_r_sate = 0
        # 设置最终robot位姿
        self.goal_x = 0.0
        self.goal_y = 10.0
        self.desire_final_speed = None
        self.desire_final_angle = None
        self.eps = 0  # 记录当前迭代轮数
        self.seed()
        self.steps = 0  # for debug

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    # fixed version
    def reset_goal(self):
        goal_num = self.np_random.randint(0, 2)
        # self.goal_x = self.cc_func(round(goal_sequence[ac_num][goal_num][0], 1))
        # self.goal_y = self.cc_func(round(goal_sequence[ac_num][goal_num][1], 1))
        self.goal_x = round(goal_sequence[ac_num][goal_num][0], 1)
        self.goal_y = round(goal_sequence[ac_num][goal_num][1], 1)

    def compute_distance(self, lhx, lhy, rhx, rhy):
        return math.sqrt((lhx - rhx) ** 2 + (lhy - rhy) ** 2)

    # 使用相邻两次移动计算robot朝向,移动距离很小的话（特别有噪声的情况下,怎么可能计算的准确？需要借助IMU?）
    def compute_polar_coordinates(self):
        edge_a = self.compute_distance(self.last_robot_pose_x, self.last_robot_pose_y,
                                       self.gazebo_robot_pose_x, self.gazebo_robot_pose_y)
        edge_b = self.compute_distance(self.last_robot_pose_x, self.last_robot_pose_y, self.goal_x, self.goal_y)
        edge_c = self.compute_distance(self.gazebo_robot_pose_x, self.gazebo_robot_pose_y, self.goal_x, self.goal_y)
        cos_angle_beta = (edge_a**2 + edge_c**2 - edge_b**2)/(2*edge_a*edge_c)
        self.state_distance = edge_c  # 数值需进行处理
        self.state_angle = math.pi - math.acos(cos_angle_beta)  # 数值需进行处理

    # def distance_to_obstacle(self, num_x, num_y, count):
    #     return math.sqrt((self.obstacle_pose[count][0] - num_x) ** 2 + (self.obstacle_pose[count][1] - num_y) ** 2)

    def state_speed_wrapper(self):
        self.state_speed = round(self.gazebo_robot_speed, 4) * 200
        self.state_speed = int(self.state_speed)  # (0~100)
        self.state_angle = round(self.gazebo_robot_angle, 4) * 100
        self.state_angle = int(self.state_angle) + 50  # (0~100)

    def reset(self):
        self.steps = 0  # for debug
        self.eps += 1
        self.done = False
        # 每100轮random一次
        if self.eps % 100 == 0 or self.eps == 1:
            self.reset_goal()
            # robot 的初始朝向也随机化
            self.reset_robot_state.pose.orientation.z = self.np_random.uniform(low=-0.9, high=0.9)
            self.reset_robot_state.pose.orientation.w = self.np_random.uniform(low=-0.9, high=0.9)
            print('actor'+str(ac_num)+' --> x: ' + str(self.goal_x) + ' y: ' + str(self.goal_y))
        # 重置robot的 speed angle 注意各步骤执行顺序
        self.cmd_twist.linear.x = 0.0
        self.cmd_twist.angular.z = 0.0
        # 将robot重置回原位置
        start_time = time.time()
        while (time.time() - start_time) < 0.5:  # 这里不使用srv直接重置,
            self.act_pub.publish(self.cmd_twist)  # 先重置速度
            self.pub.publish(self.reset_robot_state)
            time.sleep(0.05)
        # time.sleep(0.5)  # 重置后需要一定时间等待激光等数据更新...
        # 重置state中的pose speed angle
        self.gazebo_robot_pose_x, self.gazebo_robot_pose_y, self.gazebo_robot_speed, self.gazebo_robot_angle = \
            self.get_gazebo_state()
        self.gazebo_robot_speed = 0.0
        self.gazebo_robot_angle = 0.0
        self.laser_data = self.thread1.laser_data
        self.last_robot_pose_x = self.gazebo_robot_pose_x
        self.last_robot_pose_y = self.gazebo_robot_pose_y
        self.sample_fuc()
        # self.get_rid_of_bad_data()
        self.int_sample_fuc()
        # reset 函数需要提供初始状态
        self.state = np.array(self.int_sample_laser_data[:])
        # self.state = np.array([])  # 用来只有目标点作为状态时
        self.state_speed_wrapper()
        self.state = np.append(self.state, [int(self.goal_x*10), int(self.goal_y*10),
                                            int(self.gazebo_robot_pose_x*10), int(self.gazebo_robot_pose_y*10),
                                            self.state_speed, self.state_angle])
        self.sequence_state_wrap()
        self.last_distance_to_goal = self.distance_to_goal()
        # self.last_laser_data = self.sample_laser_data[:]
        self.s_state = self.s_state.reshape((64,))
        # self.state = self.state.reshape((4,))
        return self.s_state

    # 连续4个state合在一起
    def sequence_state_wrap(self):
        if self.steps is 0:
            self.s_state = self.state
            for i in range(0, 3):
                self.s_state = np.append(self.s_state, self.state)
        else:
            for j in range(0, 3):
                for k in range(0, 16):
                    self.s_state[j*16+k] = self.s_state[(j+1)*16+k]
            for l in range(0, 16):
                self.s_state[3 * 16 + l] = self.state[l]

    def step(self, action):
        self.steps += 1
        assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))
        # robot 的加速度是有限制的,这里的选取一定要慎重,有两种方案,要么把变化数值都设成很小(不宜,难以快速响应环境)
        # 要么将publish的频率设限,20hz(+0.1情况下)以下
        if action == 0:
            self.gazebo_robot_speed += 0.10
        elif action == 1:
            self.gazebo_robot_speed -= 0.10
        elif action == 2:
            self.gazebo_robot_angle = 0.20
        elif action == 3:
            self.gazebo_robot_angle = -0.20
        elif action == 4:
            self.gazebo_robot_angle = 0.0
        elif action == 5:
            pass  # 保持当前状态
        else:
            gym.logger.warn("Unknown situation !")

        self.gazebo_robot_speed = np.clip(self.gazebo_robot_speed, self.min_speed, self.max_speed)
        self.gazebo_robot_angle = np.clip(self.gazebo_robot_angle, self.min_angle, self.max_angle)

        # 与许多其它os不同之处,p2os设置好cmd_vel后,robot会一直保持该速度,
        # self.cmd_twist.linear.x = 0
        # gazebo中robot就算什么指令都没有,还是会有非常小的cmd_twist.linear.x
        # 与cmd_twist.angular.z,如果直接publish这些数据,只会造成误差越来越大
        if abs(self.gazebo_robot_speed) < 0.025:
            self.gazebo_robot_speed = 0.0
        if abs(self.gazebo_robot_angle) < 0.025:
            self.gazebo_robot_angle = 0.0
        self.cmd_twist.linear.x = self.gazebo_robot_speed
        self.cmd_twist.angular.z = self.gazebo_robot_angle
        # attention! testing!
        # self.cmd_twist.linear.x = 0.5
        # self.cmd_twist.angular.z = 0.0
        time.sleep(0.08)  # 0.075
        self.act_pub.publish(self.cmd_twist)
        # 记录部分上一轮数据
        self.laser_data = self.thread1.laser_data
        self.last_robot_pose_x = self.gazebo_robot_pose_x
        self.last_robot_pose_y = self.gazebo_robot_pose_y
        # 执行完动作,检查一下状态值(这里的严谨性有待考察,因为这里并不是严格的action一下,返回一个状态)
        self.gazebo_robot_pose_x, self.gazebo_robot_pose_y, self.gazebo_robot_speed, self.gazebo_robot_angle = \
            self.get_gazebo_state()
        self.sample_fuc()
        # self.get_rid_of_bad_data()
        self.int_sample_fuc()
        self.state = np.array(self.int_sample_laser_data[:])
        # self.state = np.array([])
        self.state_speed_wrapper()
        self.state = np.append(self.state, [int(self.goal_x*10), int(self.goal_y*10),
                                            int(self.gazebo_robot_pose_x*10), int(self.gazebo_robot_pose_y*10),
                                            self.state_speed, self.state_angle])
        # 应该把最后的位姿信息也作为限制, min(distance) 选择0.25,有待考察,因为将测距仪放在车头前部
        if self.distance_to_goal() < self.distance_to_goal_threshold \
                or min(self.sample_laser_data) < self.collision_threshold:  # and (self.gazebo_robot_speed < 0.1):
            self.done = True
            if self.steps <= 5:  # for debug
                print("robot: "+str(ac_num)+" : ")
                print(self.sample_laser_data)
                print("x: "+str(self.gazebo_robot_pose_x)+"y: "+str(self.gazebo_robot_pose_y))
        self.sequence_state_wrap()
        self.s_state = self.s_state.reshape((64, ))
        # self.state = self.state.reshape((4, ))
        reward_num = self.reward()  # reward值必须在更新last_distance_to_goal之前计算
        self.last_distance_to_goal = self.distance_to_goal()  # 更新距离,为下次计算reward准备
        # self.last_laser_data = self.sample_laser_data[:]  # 注意这里,坑....  copy!!!
        return self.s_state, reward_num, self.done, {}

    def distance_to_goal(self):
        return math.sqrt((self.goal_x - self.gazebo_robot_pose_x)**2 + (self.goal_y - self.gazebo_robot_pose_y)**2)

    def reward(self):
        # 应该把最后的位姿信息也作为reward的限制,与done保持一致,目标距离与障碍物距离之间应该有交叉reward?
        if self.distance_to_goal() < self.distance_to_goal_threshold:
            return self.arrive_reward
        elif min(self.sample_laser_data) < self.collision_threshold:  # 有关障碍物距离部分,原始激光数据有噪声（或其它原因,不能直接做判定,甚至达到0.2 ???）
            return self.collision_reward
        # 谨慎选择有关距离的reward_fuc, 避免robot有可能出现的刷分行为(与arrive_reward的比例,正负等等)
        # 有关距离目标距离的部分,应该可以加快学习速度,但是比较难处理,超过目标点之后,跑远了就给它负的reward
        # 方案一
        # else:
        #     return self.road_reward_times * (self.last_distance_ro_goal-self.distance_to_goal())
        # 方案二
        else:
            if self.last_distance_to_goal - self.distance_to_goal() > 0:  # 越来越近
                return 1.0*self.road_reward_times
            else:  # 现在这样原地不动,也是负reward, >=则无惩罚
                return -1.0*self.road_reward_times  # 可以加大远离的惩罚
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
        # return self.cc_func(robot_pose_x), self.cc_func(robot_pose_y), robot_twist_linear_x, robot_twist_angular_z
        return robot_pose_x, robot_pose_y, robot_twist_linear_x, robot_twist_angular_z

    # 从原激光数据中抽取10组0.0型的激光数据,相当于每20°取一个;
    # 为了各robot可以互相探测,激光安装在车身前部,但更合理的做法应当是车身中央,所以要将测量到的laser数据进行一下变换0.15
    def sample_fuc(self):
        self.sample_laser_data[0] = self.laser_data[0]
        self.sample_laser_data[-1] = self.laser_data[-1]
        for i in range(1, 9):
            self.sample_laser_data[i] = self.laser_data[i * 72]
        for j in range(0, 10):
            temp_angle = (math.pi / 9) * j  # 弧度制
            self.sample_laser_data[j] += math.sin(temp_angle) * 0.15  # 修正中心位置
            # 离散化,保留1位小数,注意round()既非截断,也非四舍五入,也许2位更好？
            self.sample_laser_data[j] = round(self.sample_laser_data[j], 1)  
            if self.sample_laser_data[j] > 10.0:
                self.sample_laser_data[j] = 10.0


    def int_sample_fuc(self):
        for j in range(0, 10):
            self.int_sample_laser_data[j] = int(self.sample_laser_data[j]*10)  # 变成整数

    # def __del__(self):
    #     # 等待至线程中止。这阻塞调用线程直至线程的join()方法被调用中止-正常退出或者抛出未处理的异常-或者是可选的超时发生。
    #     self.thread1.join()  # 还是不对
    #
    def close(self):
        pass
        # self.thread1.join()  # 还是不对
