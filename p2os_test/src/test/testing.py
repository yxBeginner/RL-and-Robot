import rospy
import math
import time
from geometry_msgs.msg import Twist

ac_num = 1


def main():
    rospy.init_node("testing_ori")
    act_pub = rospy.Publisher('/robot' + str(ac_num) + '/pioneer/cmd_vel', Twist, queue_size=1)
    cmd_twist = Twist()
    cmd_twist.linear.x = 0.2
    cmd_twist.angular.z = 0.0
    start_time = time.time()
    while (time.time() - start_time) < 0.5:  # 这里不使用srv直接重置,
        act_pub.publish(cmd_twist)
        time.sleep(0.05)


if __name__ == '__main__':
    main()
