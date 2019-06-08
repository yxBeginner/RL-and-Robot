#!/usr/bin/python3
import time
import math
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState
from std_srvs.srv import Empty
# from gazebo_msgs.srv import GetModelStateResponse
# from rosgraph_msgs.msg import Clock

ac_num = 1
goal_x = 8.0
goal_y = 8.0


def laser_callback(msg):
    laser_data = msg.ranges


def compute_distance(lhx, lhy, rhx, rhy):
    return math.sqrt((lhx - rhx) ** 2 + (lhy - rhy) ** 2)


def cpose_goal_angle(gazebo_robot_pose_y, gazebo_robot_pose_x):
    return math.atan2(goal_y - gazebo_robot_pose_y, goal_x - gazebo_robot_pose_x)  # (-pi,pi]


def get_gazebo_state(get_state_srv):
    # 第一个参数model_name,第二个参数为relative_entity_name
    model_state = get_state_srv('robot'+str(ac_num), '')  # 注意model_state 的类型为GetModelStateResponse
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
    robot_to_goal_angle = cpose_goal_angle(robot_pose_y, robot_pose_x) - robot_polar_angle  # 这里会被计算成（-2pi,2pi]
    if robot_to_goal_angle > math.pi:  # 整合为(-pi,pi]
        robot_to_goal_angle -= 2*math.pi  # robot_polar_angle是robot相对默认坐标系的转向角
    elif robot_to_goal_angle <= -math.pi:
        robot_to_goal_angle += 2*math.pi
    # print("robot 当前与坐标轴的角度")
    # print(robot_polar_angle / math.pi * 180)
    # print("goal 与当前robot坐标轴的角度")
    # print(self.cpose_goal_angle() / math.pi * 180)
    # print("goal 与当前robot的角度")
    # print(robot_to_goal_angle/math.pi * 180)
    # robot_polar_angle = round(robot_polar_angle*10, 1)  # normalization 注释掉这句话
    state_polar_angle = math.pi + robot_to_goal_angle  # 变为(0,2pi]
    state_polar_angle = round(state_polar_angle*10, 1)  # normalization 注释掉这句话
    print("state_angle:")
    print(state_polar_angle)
    # robot_orientation_z = round(50*(model_state.pose.orientation.z+1), 1)  # z与w 在[-1,1]之间
    # robot_orientation_w = round(50*(model_state.pose.orientation.w+1), 1)
    # return self.cc_func(robot_pose_x), self.cc_func(robot_pose_y), robot_twist_linear_x, robot_twist_angular_z
    return robot_pose_x, robot_pose_y, robot_twist_linear_x, robot_twist_angular_z, robot_to_goal_angle


def main():
    rospy.init_node("read_robot_state_node" + str(ac_num))
    sub_name = '/robot' + str(ac_num) + '/laser/scan'
    laser_sub = rospy.Subscriber(sub_name, LaserScan, laser_callback, queue_size=1)

    # 获取状态服务
    rospy.wait_for_service('gazebo/get_model_state')
    get_state_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    new_obstacle_state = ModelState()
    new_obstacle_state.model_name = "robot"+str(ac_num)

    start_time = time.time()
    while (time.time() - start_time) < 60:  # 这里不使用srv直接重置,
        gazebo_robot_pose_x, gazebo_robot_pose_y, gazebo_robot_speed, gazebo_robot_angle, state_polar_angle = \
            get_gazebo_state(get_state_srv)
        # print(gazebo_robot_speed)
        print("distance:")
        print(compute_distance(goal_x, goal_y, gazebo_robot_pose_x, gazebo_robot_pose_y))
        print("angle:")
        print(state_polar_angle / math.pi * 180)
        # print(state_polar_angle)
        time.sleep(1)


if __name__ == '__main__':
    main()
