#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import time
import math
from tf.transformations import euler_from_quaternion

# 存储上一次的里程计信息
last_odometry = None
cur_odometry = None

def odom_callback(msg):
    global cur_odometry
    cur_odometry = {
        'secs': msg.header.stamp.secs,
        'nsecs': msg.header.stamp.nsecs,
        'pose': msg.pose.pose,
        'Twist': msg.twist.twist
    }

def scan_callback(data):
    global last_odometry
    global cur_odometry
    # 检查是否已经接收到了里程计信息
    if last_odometry is None:
        last_odometry = cur_odometry
        rospy.loginfo("Waiting for odometry message...")
        return
    last_pose = last_odometry['pose']
    cur_pose = cur_odometry['pose']
    last_orientation = last_pose.orientation
    cur_orientation = cur_pose.orientation
    # 计算时间差
    current_time = data.header.stamp.secs + data.header.stamp.nsecs / 1e9
    last_time = last_odometry['secs'] + last_odometry['nsecs'] / 1e9
    time_diff = current_time - last_time

    # 提取上一次和当前的位置和方向
    last_position = last_pose.position
    cur_position = cur_pose.position

    last_orientation_quat = [last_orientation.x, last_orientation.y, last_orientation.z, last_orientation.w]
    cur_orientation_quat = [cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z, cur_pose.orientation.w]

    # 计算线位移
    linear_displacement = math.sqrt((cur_position.x - last_position.x)**2 + (cur_position.y - last_position.y)**2 + (cur_position.z - last_position.z)**2)
    linear_displacement_mm = linear_displacement * 1000  # 转换为毫米

    # 计算角位移
    last_euler = euler_from_quaternion(last_orientation_quat)
    cur_euler = euler_from_quaternion(cur_orientation_quat)
    angular_displacement = cur_euler[2] - last_euler[2]  # 假设只考虑绕Z轴的旋转
    angular_displacement_degrees = math.degrees(angular_displacement)  # 转换为度

    # 计算时间戳（以毫秒为单位）
    timestamp = int(time.time() * 1000)

    # 将激光雷达的距离数据转换为毫米单位
    distances_mm = [int(distance * 1000) if distance != float('inf') else 11000 for distance in data.ranges]

    # 格式化数据
    data_str = "{} {} {} {}\n".format(
        time_diff,  # 时间差（秒）
        linear_displacement_mm,  # 线位移（毫米）
        angular_displacement_degrees,  # 角位移（度）
        ' '.join(map(str, distances_mm))
    )

    # 写入文件
    with open("laser_data.dat", "a") as file:
        file.write(data_str)

    # 更新上一次的里程计信息
    last_odometry = cur_odometry

def listener():
    rospy.init_node('laser_scan_listener', anonymous=True)
    rospy.Subscriber("/tianracer/scan", LaserScan, scan_callback)
    rospy.Subscriber("/tianracer/odom", Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()