# import rospy
# from gazebo_msgs.srv import GetModelState
# from gazebo_msgs.srv import GetModelStateResponse
#
#
# rospy.init_node("test0624")
# print("test1")
# rospy.wait_for_service('gazebo/get_model_state')
# get_state_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
# # model_state = GetModelState()
# print("test3")
# rate = rospy.Rate(2)
#
# while not rospy.is_shutdown():
#     model_state = get_state_srv('robot_description', '')  # 注意model_state 的类型为GetModelStateResponse
#     # test_state = GetModelStateResponse()
#     # test_state = model_state
#     print(model_state.pose.position.x)
#     rate.sleep()
#
# # print(model_state)
# import math
# import numpy as np
# import gym
#
# masspole = 0.1
# length = 0.5  # actually half the pole's length
# polemass_length = (masspole * length)
# force_mag = 10.0
# tau = 0.02  # seconds between state updates
#
# # Angle at which to fail the episode
# theta_threshold_radians = 12 * 2 * math.pi / 360
# x_threshold = 2.4
#
# # Angle limit set to 2 * theta_threshold_radians so failing observation is still within bounds
# high = np.array([
#     x_threshold * 2,
#     np.finfo(np.float32).max,
#     theta_threshold_radians * 2,
#     np.finfo(np.float32).max])
#
# # state = self.np_random.uniform(low=-0.05, high=0.05, size=(4,))
# action_space = gym.spaces.Discrete(2)
# observation_space = gym.spaces.Box(-high, high)
# print(action_space.shape)
# print(observation_space.shape)
#
# rew = 10.0
# episode_rewards = [0.0]
# episode_rewards[-1] += rew
# print(episode_rewards)

#list_a = [x for x in range(0, 10)][i for i in range(0, 10)]
#print(list_a)

from collections import deque
# a = 1.2355
# b = round(2.635, 2)
# c = round(1, 3)
# print(round(a, 3))
# print(b)
# print(c)
# num_rew_sample_laser_data = 3
# rew_sample_laser_data = [[] for x in range(0, num_rew_sample_laser_data)]
# print(rew_sample_laser_data)
# rew_sample_laser_data[1] = [1,2,3]
# print(rew_sample_laser_data)
# import numpy as np
# obstacle_pose = [[0 for x in range(0, 2)] for x in range(0, 20)]
# print(obstacle_pose)
#
# for i in range(0, 20):
#     obstacle_pose[i][0] = (20 * (np.random.ranf() - 0.5))  # x~(-10,10)
#     while abs(obstacle_pose[i][0]) < 1.0:
#         obstacle_pose[i][0] = (20 * (np.random.ranf() - 0.5))  # 距离初始位置过近
#     obstacle_pose[i][0] = round(obstacle_pose[i][0], 1)  # 离散化
#     obstacle_pose[i][1] = (20 * (np.random.ranf() - 0.5))  # y~(-10,10)
#     while abs(obstacle_pose[i][1]) < 1.0:
#         obstacle_pose[i][1] = (20 * (np.random.ranf() - 0.5))  # 距离初始位置过近
#     obstacle_pose[i][1] = round(obstacle_pose[i][1], 1)
#
# print(obstacle_pose)


# eps = 100
# print(eps % 10 == 0)
# eps = 112
# print(eps % 10 == 0)
# sample_laser_data = float('inf')
# if sample_laser_data == float('inf'):
#     print('true')
# else:
#     print('false')

# gazebo_robot_speed = 0.435667
# state_speed = round(gazebo_robot_speed, 4) * 100
# print(state_speed)
# # state_speed = int(round(state_speed, 1))   # 精度更高些(0~100)
# state_speed = int(state_speed)  # 精度更高些(0~100)
# print(state_speed)

import math

goal_y = 2
goal_x = 2
gazebo_robot_pose_y = 1
gazebo_robot_pose_x = -1
# pi_theta = math.atan2(goal_y, goal_x)  # (-pi,pi]
pi_theta = math.atan2(goal_y - gazebo_robot_pose_y, goal_x - gazebo_robot_pose_x)
print(pi_theta)
print(pi_theta/math.pi * 180)
# robot_polar_angle = 1.5 * math.pi
# if robot_polar_angle > math.pi:  # 整合为[0,2pi]
#     robot_polar_angle -= 2 * math.pi  # robot_polar_angle是robot相对默认坐标系的转向角
# print(robot_polar_angle)
# print(math.pi)
# print(math.acos(0.5)*180/math.pi)
