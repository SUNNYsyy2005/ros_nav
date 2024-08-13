import rospy
from ackermann_msgs.msg import AckermannDriveStamped

class Action(object):
    def __init__(self):
        rospy.init_node('action_node', anonymous=True)
        self.pub = rospy.Publisher('/tianracer/ackermann_cmd_stamped', AckermannDriveStamped, queue_size=10)

    def sendCommand(self, vx=0, vw=0):
        ackermann_cmd = AckermannDriveStamped()
        ackermann_cmd.header.stamp = rospy.Time.now()
        ackermann_cmd.drive.speed = vx
        ackermann_cmd.drive.steering_angle = vw
        self.pub.publish(ackermann_cmd)

if __name__ == '__main__':
    action_module = Action()
    rate = rospy.Rate(50)  # 50Hz
    while not rospy.is_shutdown():
        action_module.sendCommand(vx=100, vw=10)
        rate.sleep()