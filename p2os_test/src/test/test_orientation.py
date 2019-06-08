import rospy
import math
import time
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist

ac_num = 1
# theta = -30
# pi_theta = theta/180 * math.pi
# print(pi_theta)
# # pi_theta = 3.92
# z = math.sin(pi_theta/2)
# w = math.cos(pi_theta/2)
# # print(z)
# # print(w)
# # theta = 0时, z=0 w=1
# # theta<0时,z<0
# # w = -0.3796838457960443
# # print(-0.7784073464102067-3.15)
# pi_theta_0 = 2 * math.acos(w)
# pi_theta_1 = 2 * math.asin(w)
# print(pi_theta_0)
# print(pi_theta_1)
# print(pi_theta/math.pi * 180)
# print("attention")
# print(pi_theta_0/math.pi * 180)
# print(pi_theta_1/math.pi * 180)
z = 0
w = 0

def main():
    rospy.init_node("testing_ori")
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

    rospy.wait_for_service('gazebo/get_model_state')
    get_state_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    rospy.wait_for_service('/gazebo/reset_simulation')
    reset_srv = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

    act_pub = rospy.Publisher('/robot' + str(ac_num) + '/pioneer/cmd_vel', Twist, queue_size=1)
    cmd_twist = Twist()

    reset_srv()

    new_obstacle_state = ModelState()
    new_obstacle_state.model_name = "robot"+str(ac_num)
    # cc_x = 0.0  # coordinate correction
    # cc_y = 0.0  # 使用其它的坐标方案

    if ac_num == 1:
        new_obstacle_state.pose.position.x = 5.0
        new_obstacle_state.pose.position.y = 5.0
    new_obstacle_state.pose.position.z = 0
    # 注意４元数的概念
    new_obstacle_state.pose.orientation.x = 0  # z与w 控制了水平朝向？
    new_obstacle_state.pose.orientation.y = 0
    new_obstacle_state.pose.orientation.z = z
    new_obstacle_state.pose.orientation.w = w

    new_obstacle_state.twist.linear.x = 0
    new_obstacle_state.twist.linear.y = 0
    new_obstacle_state.twist.linear.z = 0

    new_obstacle_state.twist.angular.x = 0
    new_obstacle_state.twist.angular.y = 0
    new_obstacle_state.twist.angular.z = 0

    new_obstacle_state.reference_frame = "world"

    start_time = time.time()
    while (time.time() - start_time) < 0.5:  # 这里不使用srv直接重置,
        pub.publish(new_obstacle_state)
        time.sleep(0.05)

    # 第一个参数model_name,第二个参数为relative_entity_name
    model_state = get_state_srv('robot' + str(ac_num), '')  # 注意model_state 的类型为GetModelStateResponse
    robot_pose_x = model_state.pose.position.x  # 注意pose为(x,y,z) , 离散,
    robot_pose_y = model_state.pose.position.y
    robot_orientation_z = model_state.pose.orientation.z
    robot_orientation_w = model_state.pose.orientation.w

    print(robot_pose_x)
    print(robot_pose_y)
    print(robot_orientation_z)
    print(robot_orientation_w)

    cmd_twist.linear.x = 0.5
    cmd_twist.angular.z = 0.0

    act_pub.publish(cmd_twist)  # 先重置速度


if __name__ == '__main__':
    main()
