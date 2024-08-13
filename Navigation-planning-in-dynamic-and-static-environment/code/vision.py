import rospy
from sensor_msgs.msg import LaserScan
from LaserData import AMCLLaserData  # 假设这个类可以从LaserScan消息更新数据

class Vision(object):
    def __init__(self):
        self._my_robot = Robot(id=0)
        self._my_robot.visible = True
        self._my_robot.x = 1000
        self._my_robot.y = 1000
        self._my_robot.vel_x = 0
        self._my_robot.vel_y = 0
        self._my_robot.orientation = 9
        self.data = AMCLLaserData()
        # 订阅tianracer/scan话题
        rospy.init_node('vision_node', anonymous=True)
        rospy.Subscriber('tianracer/scan', LaserScan, self.scan_callback)

    def scan_callback(self, msg):
        # 假设AMCLLaserData类有一个方法可以从LaserScan消息更新数据
        self.data.update_from_scan(msg)

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