#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

class LaserListener:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('laser_listener', anonymous=True)
        
        # 创建一个订阅者，订阅/tianracer/laser主题
        self.laser_sub = rospy.Subscriber('/tianracer/scan', LaserScan, self.laser_callback)
        
        # 用于存储最新的激光雷达数据
        self.latest_laser_data = None
        
        # 设置定时器，每1秒调用一次self.print_latest_data函数
        rospy.Timer(rospy.Duration(1), self.print_latest_data)

    def laser_callback(self, data):
        # 每当接收到新的激光雷达数据时，更新self.latest_laser_data
        self.latest_laser_data = data

    def print_latest_data(self, event):
        # 如果有最新的激光雷达数据，打印出来
        if self.latest_laser_data is not None:
            rospy.loginfo(len(self.latest_laser_data.ranges))

if __name__ == '__main__':
    listener = LaserListener()
    listener.__init__()
    rospy.spin()


    #rosrun tianracer_gazebo 2.py