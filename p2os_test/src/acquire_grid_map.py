# 接收概率占据栅格地图的raw data, 并将其转换为一个二维矩阵
import rospy
import time
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.srv import GetMap

map_data = None


def map_callback(msg):
    global map_data
    map_data = msg
    print(map_data)


def set_map():
    rospy.init_node("acquire_map_node")
    # 获取地图
    # rospy.wait_for_service('gazebo/get_model_state')
    # get_map_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetMap)
    map_sub = rospy.Subscriber('/map', OccupancyGrid, map_callback, queue_size=10)
    print(map_data)

    # global map_data
    # md = MapMetaData()
    # md = map_data.info
    #
    # print(md.width)


class GetMapNode(object):
    def __init__(self):
        rospy.init_node("acquire_map_node")
        # self.map_data = OccupancyGrid()
        self.occ_grid_map = None

    def map_callback(self, msg):
        self.occ_grid_map = msg

    def run(self):
        time_a = time.time()
        while time.time() - time_a < 3:
            map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=10)

    def reset_to_matrix(self):
        w = self.occ_grid_map.info.width
        h = self.occ_grid_map.info.height
        re_map_matrix = [[0 for i in range(0, w)] for _ in range(0, h)]
        for i in range(0, h):
            for j in range(0, w):
                re_map_matrix[i][j] = self.occ_grid_map.data[j+i*w]
        return re_map_matrix


if __name__ == "__main__":
    a = GetMapNode()
    a.run()
    a.reset_to_matrix()
    # print(a.occ_grid_map)
