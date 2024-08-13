import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D  # 修改为导入Pose2D消息类型
from LaserData import AMCLLaserData

class Vision(object):
    def __init__(self):
        self._my_robot = Robot(id=0)
        self._my_robot.visible = True
        self._my_robot.x = 1000
        self._my_robot.y = 975
        self._my_robot.vel_x = 0
        self._my_robot.vel_y = 0
        self._my_robot.orientation = 0
        self.data = AMCLLaserData()
        rospy.Subscriber('tianracer/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('tianracer/pose', Pose2D, self.pose_callback)  # 修改为订阅Pose2D消息

    def scan_callback(self, msg):
        self.data.update_from_scan(msg)

    def pose_callback(self, msg):
        # 根据Pose2D消息结构更新self._my_robot的x、y和orientation值
        self._my_robot.x = msg.x
        self._my_robot.y = msg.y
        self._my_robot.orientation = msg.theta  # Pose2D包含x, y和theta（orientation）

    @property
    def my_robot(self):
        return self._my_robot

    @my_robot.setter
    def my_robot(self, value):
        self._my_robot = value

class Robot(object):
    def __init__(self, id, visible=False,
                x=-999999, y=-999999, vel_x=0, vel_y=0, orientation=0,
                raw_x=-999999, raw_y=-999999, raw_vel_x=0, raw_vel_y=0, raw_orientation=0):
        self.id = id
        self.visible = visible
        self.x = x
        self.y = y
        self.vel_x = vel_x
        self.vel_y = vel_y
        self.orientation = orientation
        self.raw_x = raw_x
        self.raw_y = raw_y
        self.raw_vel_x = raw_vel_x
        self.raw_vel_y = raw_vel_y
        self.raw_orientation = raw_orientation

if __name__ == '__main__':
    rospy.init_node('vision_node', anonymous=True)
    vision_module = Vision()
    rospy.spin()  # 保持节点运行，直到被关闭