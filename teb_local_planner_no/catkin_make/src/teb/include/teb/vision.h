#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h> // 导入Pose2D消息类型
#include "laserdata.h" // 假设有一个对应的AMCLLaserData类
class Robot {
    public:
        int id;
        bool visible;
        double x, y, vel_x, vel_y, orientation;
        double raw_x, raw_y, raw_vel_x, raw_vel_y, raw_orientation;

        Robot(){
            this->id=0;
            this->visible=false;
            this->x=-999999; 
            this->y=-999999;
            this->vel_x=0;
            this->vel_y=0;
            this->orientation=0;
            this->raw_x=-999999;
            this->raw_y=-999999;
            this->raw_vel_x=0;
            this->raw_vel_y=0;
            this->raw_orientation=0;
        } 
    };
class Vision {
public:
    Vision(){
        _my_robot.visible = true;
        _my_robot.x = 400;
        _my_robot.y = 400;
        _my_robot.vel_x = 0;
        _my_robot.vel_y = 0;
        _my_robot.orientation = 1.57;
        scan_sub = nh.subscribe("tianracer/scan", 1000, &Vision::scanCallback, this);
        pose_sub = nh.subscribe("tianracer/pose", 1000, &Vision::poseCallback, this);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        data.update_from_scan(*msg);
    }

    void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
        _my_robot.x = msg->x;
        _my_robot.y = msg->y;
        _my_robot.orientation = msg->theta; // Pose2D包含x, y和theta（orientation）
        //ROS_INFO("x: %f, y: %f, theta: %f", _my_robot.x, _my_robot.y, _my_robot.orientation);
    }

public:
    Robot _my_robot;
    AMCLLaserData data;
    ros::Subscriber scan_sub, pose_sub;
    ros::NodeHandle nh;
};

/* int main(int argc, char **argv) {
    ros::init(argc, argv, "vision_node");
    Vision vision_module;
    ros::spin(); // 保持节点运行，直到被关闭

    return 0;
} */