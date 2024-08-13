#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped

def backup():
    # 初始化ROS节点
    rospy.init_node('backup_behavior')

    # 创建一个发布Ackermann驱动命令的发布器
    pub = rospy.Publisher('/ackermann_cmd_stamped', AckermannDriveStamped, queue_size=1)

    # 创建一个Ackermann驱动命令
    drive_msg = AckermannDriveStamped()
    drive_msg.drive.speed = -0.1  # 后退速度
    drive_msg.drive.steering_angle = 0.0  # 转向角度

    # 发布Ackermann驱动命令
    pub.publish(drive_msg)

    # 等待一段时间
    rospy.sleep(1.0)

    # 停止小车
    drive_msg.drive.speed = 0.0
    pub.publish(drive_msg)

if __name__ == '__main__':
    try:
        backup()
    except rospy.ROSInterruptException:
        pass