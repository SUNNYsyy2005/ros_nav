import rospy
from geometry_msgs.msg import Twist

class Action(object):
    def __init__(self):
        self.pub = rospy.Publisher('/tianracer/cmd_vel', Twist, queue_size=1000)
        self.last_call_time = rospy.get_time()  # 初始化上次调用时间
        self.steer_angle = 0
        self.last_vm = False
        self.last_vw = 0

    def sendCommand(self, vx=0, vw=0):
        current_time = rospy.get_time()
        interval = current_time - self.last_call_time  # 计算时间间隔
        self.last_call_time = current_time  # 更新上次调用时间
        if self.last_vm:
            self.steer_angle += vw * interval
            print("steer_angle: ", self.steer_angle)
        else:
            self.steer_angle = 0
        if abs(vx) < 1e-6:
            self.last_vm = True
            self.last_vw = vw
        else:
            self.last_vm = False

        twist_cmd = Twist()
        twist_cmd.linear.x = vx
        twist_cmd.angular.z = vw

        # 打印消息内容和调用间隔
        print("Interval: {:.2f} seconds, Publishing Twist Command: linear.x={}, angular.z={}".format(interval, twist_cmd.linear.x, twist_cmd.angular.z))
        self.pub.publish(twist_cmd)

if __name__ == '__main__':
    rospy.init_node('action_node', anonymous=True)  # 初始化节点
    action_module = Action()
    rate = rospy.Rate(50)  # 50Hz
    while not rospy.is_shutdown():
        action_module.sendCommand(vx=100, vw=10)
        rate.sleep()