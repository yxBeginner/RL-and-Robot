import rospy
from sensor_msgs.msg import LaserScan
# import time

# LaserScan 数据格式
# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# float32 angle_min
# float32 angle_max
# float32 angle_increment
# float32 time_increment
# float32 scan_time
# float32 range_min
# float32 range_max
# float32[] ranges
# float32[] intensities
# 频率为40hz, ranges大约719个数据 , 10 * 72


# 也许设计成每秒更新10次状态？ 过高的频率意义不大？
def laser_callback(msg):
    print(msg)


rospy.init_node("receive_robot_laser_data")
laser_sub = rospy.Subscriber('/laser/scan', LaserScan, laser_callback)
rospy.spin()

# start_time = time.time()
# while (time.time()-start_time) < 0.5:
#     pub.publish(new_robot_state)


