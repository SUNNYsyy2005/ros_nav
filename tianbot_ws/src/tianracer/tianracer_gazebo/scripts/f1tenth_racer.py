#! /usr/bin/env python
# @Source: https://www.guyuehome.com/35146
# @Time: 2023/10/20 17:02:46
# @Author: Jeff Wang(Lr_2002)
# LastEditors: sujit-168 su2054552689@gmail.com
# LastEditTime: 2024-03-22 10:23:42

import os
import tf
import math
import rospy, rospkg
import actionlib # 引用 actionlib 库
from actionlib_msgs.msg import GoalStatus
from ackermann_msgs.msg import AckermannDriveStamped
import waypoint_race.utils as utils

import move_base_msgs.msg as move_base_msgs
import visualization_msgs.msg as viz_msgs

world = os.getenv("TIANRACER_WORLD", "tianracer_racetrack")

class RaceStateMachine(object):
    def __init__(self, filename, repeat=True):
        """
        init RaceStateMachine
        filename: path to your yaml file
        reapeat: determine whether to visit waypoints repeatly(usually True in 110)
        """
        self._waypoints = utils.get_waypoints(filename) # 获取一系列目标点的值

        action_name = 'move_base'
        self._ac_move_base = actionlib.SimpleActionClient(action_name, move_base_msgs.MoveBaseAction) # 创建一个 SimpleActionClient
        rospy.loginfo('Wait for %s server' % action_name)
        self._ac_move_base.wait_for_server
        self._counter = 0
        self._repeat = repeat
        self._tf_listener = tf.TransformListener()
        self._distance_threshold = 2  # 取消目标点的距离阈值
        # 以下为了显示目标点：
        self._pub_viz_marker = rospy.Publisher('viz_waypoints', viz_msgs.MarkerArray, queue_size=1, latch=True)
        self._viz_markers = utils.create_viz_markers(self._waypoints)
    def get_current_position(self):
        try:
            (trans,rot) = self._tf_listener.lookupTransform('/map', '/tianracer/base_link', rospy.Time(0))
            return trans
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None
    def calculate_distance(self, pos1, pos2):
        dx = pos1[0] - pos2['pose']['position']['x']
        dy = pos1[1] - pos2['pose']['position']['y']
        return math.sqrt(dx * dx + dy * dy)
    def move_to_next(self):
        pos = self._get_next_destination()

        if not pos:
            rospy.loginfo("Finishing Race")
            return True

        goal = utils.create_move_base_goal(pos)
        rospy.loginfo("Move to %s" % pos['name'])
        self._ac_move_base.send_goal(goal)

        # 等待结果，同时检查机器人与目标点的距离
        while not rospy.is_shutdown():
            if self._ac_move_base.wait_for_result(rospy.Duration(0.3)):
                # 如果已经到达目标点，跳出循环
                break

            # 获取机器人当前位置
            current_pos = self.get_current_position()

            # 计算与目标点的距离
            distance = self.calculate_distance(current_pos, pos)

            # 如果距离小于预设的阈值，取消目标点
            if distance < self._distance_threshold:
                rospy.loginfo("Within range, canceling goal")
                self._ac_move_base.cancel_goal()
                break

        result = self._ac_move_base.get_result()
        rospy.loginfo("Result : %s" % result)
        return False

    def _get_next_destination(self):
        """
        determine what's the next goal point according to repeat 
        """
        if self._counter == len(self._waypoints):
            if self._repeat:
                self._counter = 0
            else:
                next_destination = None
        next_destination = self._waypoints[self._counter]
        self._counter = self._counter + 1
        return next_destination

    def spin(self):
        rospy.sleep(0.3)
        self._pub_viz_marker.publish(self._viz_markers)
        finished = False
        while not rospy.is_shutdown() and not finished:
            finished = self.move_to_next()
            rospy.sleep(0.6)

if __name__ == '__main__':
    rospy.init_node('race')
    
    package_name = "tianracer_gazebo"

    # Get the package path
    try:
        pkg_path = rospkg.RosPack().get_path(package_name)

        # Construct the path to scripts directory
        filename= os.path.join(pkg_path, f"scripts/waypoint_race/{world}_points.yaml")
        print(f"yaml: {filename}")
    except rospkg.ResourceNotFound:
        rospy.logerr("Package '%s' not found" % package_name)
        exit(1)

    filename = rospy.get_param("~filename",filename)
    repeat = rospy.get_param('~repeat', True)

    m = RaceStateMachine(filename, repeat)
    rospy.loginfo('Initialized')
    # 创建一个发布Ackermann驱动命令的发布器
    m._pub = rospy.Publisher('/ackermann_cmd_stamped', AckermannDriveStamped, queue_size=1)
    m.spin()
    rospy.loginfo('Finished')
