import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import numpy as np

from rosgraph_msgs.msg import Clock
# from  gazebo_msgs.msg import
import time

rospy.init_node("rest_box_state")
# pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
rospy.wait_for_service('gazebo/set_model_state')
# set_box_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
set_obstacle_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

# 用来随机重置障碍物的位置坐标
obstacle_pose = [[0 for x in range(0, 2)] for x in range(0, 20)]
print(obstacle_pose)

for i in range(0, 20):
    obstacle_pose[i][0] = (20 * (np.random.ranf() - 0.5))  # x~(-10,10)
    while abs(obstacle_pose[i][0]) < 1.0:
        obstacle_pose[i][0] = (20 * (np.random.ranf() - 0.5))  # 距离初始位置过近
    obstacle_pose[i][0] = round(obstacle_pose[i][0], 1)  # 离散化
    obstacle_pose[i][1] = (20 * (np.random.ranf() - 0.5))  # y~(-10,10)
    while abs(obstacle_pose[i][1]) < 1.0:
        obstacle_pose[i][1] = (20 * (np.random.ranf() - 0.5))  # 距离初始位置过近
    obstacle_pose[i][1] = round(obstacle_pose[i][1], 1)


# 使用服务,重置障碍物

new_obstacle_state = ModelState()

new_obstacle_state.pose.orientation.x = 0
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
for i in range(0, 15):  # 15个方块
    print("unit_box_" + str(i))
    new_obstacle_state.model_name = "unit_box_" + str(i)
    new_obstacle_state.pose.position.x = obstacle_pose[i][0]
    new_obstacle_state.pose.position.y = obstacle_pose[i][1]
    new_obstacle_state.pose.position.z = 0

    set_obstacle_srv(new_obstacle_state)


for i in range(0, 5):  # 5个圆柱体
    new_obstacle_state.model_name = "unit_cylinder_" + str(i)
    new_obstacle_state.pose.position.x = obstacle_pose[i + 15][0]
    new_obstacle_state.pose.position.y = obstacle_pose[i + 15][1]
    new_obstacle_state.pose.position.z = 0

    set_obstacle_srv(new_obstacle_state)
