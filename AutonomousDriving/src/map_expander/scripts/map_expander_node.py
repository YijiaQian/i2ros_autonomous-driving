#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import copy

def handle_map(msg):
    # 原始地图信息
    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution
    origin = msg.info.origin

    # 计算扩展后地图的尺寸和原点
    new_width = int((200 + 100) / resolution)  # 从 -100m 到 200m
    new_height = int((400 + 100) / resolution) # 从 -100m 到 400m

    new_origin = copy.deepcopy(origin)
 
    new_origin.position.x = -100  # 原点向西移动100m
    new_origin.position.y = -100  # 原点向南移动100m
    # 创建新的地图数组，初始化为-1
    new_data = np.full((new_height, new_width), -1, dtype=int)

    # # 将原始地图数据复制到新的地图中
    offset_x = int((origin.position.x - new_origin.position.x) / resolution) + 1
    offset_y = int((origin.position.y - new_origin.position.y) / resolution) + 1

    for y in range(height):
        for x in range(width):
            new_data[y + offset_y][x + offset_x] = msg.data[y * width + x]

    # for y in range(height):
    #     for x in range(width):
    #         new_x = x + offset_x
    #         new_y = y + offset_y
    #         if 0 <= new_x < new_width and 0 <= new_y < new_height:
    #             new_data[new_y][new_x] = msg.data[y * width + x]   

    # 转换回一维数组
    new_data = new_data.ravel()

    # 创建新的OccupancyGrid消息
    new_msg = OccupancyGrid()
    new_msg.header = msg.header
    new_msg.info = msg.info
    new_msg.info.width = new_width
    new_msg.info.height = new_height
    new_msg.info.origin = new_origin
    new_msg.data = list(new_data)

    # 发布新地图
    pub.publish(new_msg)

def map_expander():
    rospy.init_node('map_expander')
    rospy.Subscriber("map_raw", OccupancyGrid, handle_map)
    rospy.spin()

if __name__ == '__main__':
    global pub
    pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)
    map_expander()
