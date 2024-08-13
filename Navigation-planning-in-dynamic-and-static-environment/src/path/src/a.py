#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry

def odom_callback(msg):
    # Process the received odometry message here
    # and convert it to Pose2D message
    pose2d_msg = Pose2D()
    # Set the pose2d_msg fields accordingly
    pose2d_msg.x = msg.pose.pose.position.x/0.05+1000
    pose2d_msg.y = -msg.pose.pose.position.y/0.05+975
    # Assuming the orientation is provided as a quaternion,
    # convert quaternion to yaw angle
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    )
    # Convert quaternion to Euler angles
    euler = tf.transformations.euler_from_quaternion(quaternion)
    pose2d_msg.theta = euler[2]  # yaw
    print(pose2d_msg.x, pose2d_msg.y, pose2d_msg.theta)
    # Publish the pose2d message
    pose_pub.publish(pose2d_msg)

if __name__ == '__main__':
    rospy.init_node('pose2d_publisher')

    # Import tf for quaternion to Euler conversion
    import tf

    # Create a subscriber for the odometry topic
    rospy.Subscriber('/tianracer/odom', Odometry, odom_callback)

    # Create a publisher for the pose2d topic
    pose_pub = rospy.Publisher('/tianracer/pose', Pose2D, queue_size=10)

    # Spin the node to receive and publish messages
    rospy.spin()