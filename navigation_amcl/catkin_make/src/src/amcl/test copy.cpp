#include <iostream>
#include "amcl/map/map.h"
#include "amcl/sensors/amcl_laser.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <chrono>
#include <cmath>
#include <vector>
#include <algorithm>
#define M_PI 3.14159265358979323846

void update_kdtree(pf_sample_set_t *set) {
    // Clear the existing KD tree
    pf_kdtree_clear(set->kdtree);

    // Re-insert the samples into the KD tree
    for (int i = 0; i < set->sample_count; i++) {
        pf_sample_t *sample = set->samples + i;
        pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
    }
}
std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
// 一个简单的随机姿态生成函数，用于初始化粒子
double global_x = 0.0; // x坐标
double global_y = 0.0; // y坐标
double global_theta = M_PI/2; // 角度
double last_theta = M_PI/2;
double steer_theta = 0.0;
double last_velocity = 0.0;
double last_steering_angle_velocity = 0.0;

geometry_msgs::Pose2D msg2;
const int N = 10;
const int M = 1;
const double ANGLE_THRESHOLD = 0.5; // 角度阈值，用于确定角度是否在预测角度的一定范围内
std::chrono::steady_clock::time_point last_update_time = std::chrono::steady_clock::now();
pf_vector_t random_pose_init(void *data) {
    pf_vector_t pose;
    // 这里假设一个均匀分布，实际应用中可能需要根据实际情况调整
    pose.v[0] = (double) rand() / RAND_MAX * 2000 - 1000.0; // x 坐标
    pose.v[1] = (double) rand() / RAND_MAX * 2000 - 1000.0; // y 坐标
    pose.v[2] = (double) rand() / RAND_MAX * 2 * M_PI - M_PI; // 角度，从 -π 到 π
    printf("Random pose: (%f, %f, %f)\n", pose.v[0], pose.v[1], pose.v[2]);
    return pose;
}
bool areVectorsIdentical(const std::vector<float>& v1, const std::vector<float>& v2, float tolerance = 1e-5) {
    if (v1.size() != v2.size()) return false;
    for (size_t i = 0; i < v1.size(); ++i) {
        if (std::fabs(v1[i] - v2[i]) > tolerance) {
            //printf("i=%d v1=%f v2=%f\n",i,v1[i],v2[i]);
            return false;
        }
    }
    return true;
}
pf_t *pf;
map_t *map;
amcl::AMCLLaser laser_sensor(10, NULL);
amcl::AMCLLaserData laser_data;
sensor_msgs::LaserScan::ConstPtr last_msg;
void updateParticlePoses() {
    // 确保有足够的位移或转向变化再更新
    if (global_theta == 0 && global_x == 0 && global_y == 0) {
        return;
    }
    for (int i = 0; i < pf->sets[pf->current_set].sample_count; i++) {
        pf_sample_t *sample = &(pf->sets[pf->current_set].samples[i]);
        sample->pose.v[0] += global_x;
        sample->pose.v[1] += global_y;
        sample->pose.v[2] += global_theta-last_theta;
    }
    global_x = 0;
    global_y = 0;
    last_theta = global_theta;
}
double toPI(double angle){
    while(angle > M_PI){
        angle -= 2*M_PI;
    }
    while(angle < -M_PI){
        angle += 2*M_PI;
    }
    return angle;
}
void process_samples(pf_t* pf) {
    std::vector<pf_sample_t*> top_samples_angle(M);
    std::vector<pf_sample_t*> top_samples_xy(N);
    std::vector<pf_sample_t*> sample_pointers(pf->sets[pf->current_set].sample_count);
    // 填充指针向量
    for (size_t i = 0; i < sample_pointers.size(); ++i) {
        sample_pointers[i] = &pf->sets[pf->current_set].samples[i];
    }

    // 找到权重最高的M个样本用于角度
    std::partial_sort_copy(sample_pointers.begin(), sample_pointers.end(),
                        top_samples_angle.begin(), top_samples_angle.end(),
                        [](const pf_sample_t* a, const pf_sample_t* b) {
                            return a->weight > b->weight;
                        });

    // 计算权重最高的M个样本的角度平均值
    double average_angle = 0;
    bool first = true;
    for (auto& sample : top_samples_angle) {
        //std::cout<< MAP_GXWX(map,sample->pose.v[0])<<" "<<MAP_GYWY(map,sample->pose.v[1])<<" "<<toPI(sample->pose.v[2])<<" "<<sample->weight<<std::endl;
        if(first){
            first = false;
            average_angle = sample->pose.v[2];
        }
        
    }
    //average_angle /= M;
    average_angle = global_theta;
    // 找到权重最高的N个样本用于x、y坐标
    std::partial_sort_copy(sample_pointers.begin(), sample_pointers.end(),
                        top_samples_xy.begin(), top_samples_xy.end(),
                        [](const pf_sample_t* a, const pf_sample_t* b) {
                            return a->weight > b->weight;
                        });

    // 从这N个样本中筛选出角度值与预测角度值在ANGLE_THRESHOLD范围内的样本
    std::vector<pf_sample_t*> filtered_samples;
    std::copy_if(top_samples_xy.begin(), top_samples_xy.end(), std::back_inserter(filtered_samples),
                [average_angle](const pf_sample_t* sample) {
                    return std::abs(sample->pose.v[2] - average_angle) <= ANGLE_THRESHOLD;
                });

    // 计算筛选出的样本的x、y坐标的平均值
    pf_vector_t average_pose = {0, 0, average_angle};
    for (auto& sample : filtered_samples) {
        average_pose.v[0] += sample->pose.v[0];
        average_pose.v[1] += sample->pose.v[1];
    }
    if (!filtered_samples.empty()) {
        average_pose.v[0] /= filtered_samples.size();
        average_pose.v[1] /= filtered_samples.size();
    }
    msg2.x = MAP_GXWX(map,average_pose.v[0]);
    msg2.y = MAP_GYWY(map,average_pose.v[1]);
    msg2.theta = average_pose.v[2];
    // 输出平均位置
    printf("Average pose: %f %f %f global theta%f\n",msg2.x,msg2.y,msg2.theta,global_theta);
}
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    auto now = std::chrono::steady_clock::now();
    // 检查自上次回调以来是否已经过了5秒
    if (std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count() < 200000) {
        // 如果没有过5秒，就直接返回，不处理这次消息
        return;
    }
    updateParticlePoses();
    // 更新上次处理消息的时间
    last_time = now;
    if (last_msg) {
        // 检查当前消息与上一次消息是否完全一致
        bool is_identical = areVectorsIdentical(msg->ranges, last_msg->ranges);

        // 如果完全一致，则不执行地图更新
        if (is_identical) {
            //std::cout << "Received identical message, skipping map update." << std::endl;
            return;
        }
    }

    // 更新上一次接收到的消息
    last_msg = msg;
    //std::cout<<msg->ranges.size()<<std::endl;
    //for(int i=0;i<msg->ranges.size();i++){
        //std::cout<<msg->ranges[i]<<" ";
    //}
    laser_data.ranges.resize(msg->ranges.size(), std::vector<double>(2)); 
    laser_data.range_count = msg->ranges.size();
    laser_data.range_max = msg->range_max;
    double range_min = msg->range_min;
    double angle_increment = msg->angle_increment;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        // 填充激光雷达数据，这里只处理了距离，角度需要您根据实际情况计算
        laser_data.ranges[i][0] = msg->ranges[i];
        laser_data.ranges[i][1] = msg->angle_min + i * angle_increment;
        //printf("range: %f angle: %f\n",laser_data.ranges[i][0]/0.05,laser_data.ranges[i][1]);
    }
    
    laser_sensor.UpdateSensor(pf, &laser_data);
    //std::cout<<"Update sensor\n";
    process_samples(pf);
    // 获取AMCL的预测位置
    /*  pf_vector_t predicted_pose;
    double total_weight = 0.0;
    double max_weight = 0.0;
    pf_vector_t max_weight_pose;
    for (int i = 0; i < pf->sets[pf->current_set].sample_count; i++) {
        pf_sample_t *sample = &(pf->sets[pf->current_set].samples[i]);
        //std::cout<<sample->pose.v[0]<<" "<<sample->pose.v[1]<<" "<<sample->pose.v[2]<<" "<<sample->weight<<std::endl;
        predicted_pose.v[0] += sample->pose.v[0] * sample->weight;
        predicted_pose.v[1] += sample->pose.v[1] * sample->weight;
        predicted_pose.v[2] += sample->pose.v[2] * sample->weight;
        total_weight += sample->weight;
        if (sample->weight > max_weight) {
            max_weight = sample->weight;
            max_weight_pose = sample->pose;
        }
    }

    // 归一化预测位置
    if (total_weight > 0) {
        predicted_pose.v[0] /= total_weight;
        predicted_pose.v[1] /= total_weight;
        predicted_pose.v[2] /= total_weight;
    }
    msg2.x = predicted_pose.v[0];
    msg2.y = predicted_pose.v[1];
    msg2.theta = predicted_pose.v[2];
    // 打印预测位置
    std::cout << "Predicted pose (x, y, theta): ("
              << MAP_GXWX(map, predicted_pose.v[0]) << ", "
              << MAP_GXWX(map, predicted_pose.v[1]) << ", "
              << predicted_pose.v[2] << ")" << std::endl;
    // 打印最大权重位置
    std::cout << "Max weight pose (x, y, theta): ("
              << MAP_GXWX(map, max_weight_pose.v[0]) << ", "
              << MAP_GXWX(map, max_weight_pose.v[1]) << ", "
              << max_weight_pose.v[2] << ")" << std::endl;  */
    //exit(-1);
}
void print(const geometry_msgs::Twist::ConstPtr& msg){
    ROS_INFO("Linear:");
    ROS_INFO("  x: %f", msg->linear.x);
    ROS_INFO("  y: %f", msg->linear.y);
    ROS_INFO("  z: %f", msg->linear.z);

    ROS_INFO("Angular:");
    ROS_INFO("  x: %f", msg->angular.x);
    ROS_INFO("  y: %f", msg->angular.y);
    ROS_INFO("  z: %f", msg->angular.z);
}
void ackermannCmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    //print(msg);
    double velocity =  last_velocity; // 车辆速度
    double steering_angle_velocity = last_steering_angle_velocity; // 转向角度
   // std::cout<<velocity<<" "<<steering_angle_velocity<<std::endl;
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::microseconds>(now - last_update_time).count()/1000000.0;
    last_update_time = now;
    last_steering_angle_velocity = msg->angular.z;
    last_velocity = msg->linear.x;
    //std::cout<<dt<<"@@@@@@@@@"<<std::endl;
    if (std::fabs(steering_angle_velocity) < 1e-6) { // 直行
        if(std::fabs(velocity) > 1e-6){
            global_x += velocity * dt * cos(global_theta);
            global_y -= velocity * dt * sin(global_theta);
            global_theta += steer_theta;
            steer_theta = 0;
        }
    } else { // 转弯
        double abs_velocity = std::fabs(velocity); // 取速度的绝对值
        if(abs_velocity <1e-6){
            steer_theta += (velocity > 0 ? 1 : -1) * steering_angle_velocity * dt;
            
        }else{
            double turning_radius = abs_velocity / steering_angle_velocity;
            double theta_change = abs_velocity * dt / turning_radius;
            global_x += turning_radius * (sin(global_theta + theta_change) - sin(global_theta));
            global_y += turning_radius * (cos(global_theta + theta_change) - cos(global_theta)); // 注意y轴向下的坐标系统
            global_theta += (velocity > 0 ? 1 : -1) * theta_change;
            global_theta += steer_theta;
            steer_theta = 0;
        }
    }
    if (std::isnan(global_x)){
        std::cout<<"Velocity: "<<velocity<<" Steering angle Velocity: "<<steering_angle_velocity<<std::endl;
        exit(0);
    }
    // 确保角度保持在合理的范围内
    global_theta = fmod(global_theta, 2 * M_PI);
    if (global_theta < 0) {
        global_theta += 2 * M_PI;
    }
    //std::cout<<"Velocity: "<<velocity<<" Steering angle Velocity: "<<steering_angle_velocity<<std::endl;
    //std::cout << "X: " << global_x << ", Y: " << global_y << ", Theta: " << global_theta << std::endl;
}




int main(int argc, char** argv){
    ros::init(argc, argv, "amcl_node");
    ros::NodeHandle nh;
    ros::Subscriber ackermann_sub = nh.subscribe("/tianracer/cmd_vel", 100000, ackermannCmdCallback);
    ros::Subscriber sub = nh.subscribe("tianracer/scan", 500, laserCallback);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose2D>("tianracer/pose", 1000);
    
    msg2.x=1000;msg2.y=1000;msg2.theta=0;
    
    map = map_alloc();
    map_load_occ(map, "map/1.pgm", 0.05,0);

    printf("%d %d %f %f\n", map->size_x, map->size_y, map->origin_x, map->origin_y);
    // 粒子滤波器的参数
    int min_samples = 50;      // 最小粒子数
    int max_samples = 1000;    // 最大粒子数
    double alpha_slow = 0.1;   // 慢衰减率
    double alpha_fast = 0.1;   // 快衰减率
    // 设置AMCL的激光雷达传感器模型
    amcl::AMCLLaser aa((size_t)500, map);
    laser_sensor = aa;
    pf_vector_t v;
    v.v[0]=0;v.v[1]=0;v.v[2]=0;
    laser_sensor.SetLaserPose(v); // 设置激光雷达在机器人坐标系中的位置
    //printf("Laser sensor created %p@\n",laser_sensor.map);
    laser_sensor.SetModelLikelihoodField(0.999, 0.001, 0.1, 10); // 激光模型参数
    //printf("Laser sensor model set\n");
    laser_data.sensor = &laser_sensor;
    // 创建粒子滤波器
    pf = pf_alloc(min_samples, max_samples, alpha_slow, alpha_fast,
                        random_pose_init, &laser_data);

    if (pf == NULL) {
        fprintf(stderr, "Failed to allocate particle filter\n");
        return -1;
    }
    //printf("Particle filter created\n");
    // 初始均值和协方差矩阵
    pf_vector_t mean = {0, 0, 0}; // 初始均值 [x, y, theta]
    pf_matrix_t cov = {16, 0, 0, 0, 9, 0 , 0, 0, M_PI*M_PI}; // 初始协方差
    // 使用高斯模型初始化粒子滤波器
    pf_init(pf, mean, cov);
    //printf("Particle filter initialized\n");
    ros::Rate loop_rate(100000);
    while (ros::ok())
    {
        pose_pub.publish(msg2);
        ros::spinOnce();
        loop_rate.sleep();
        msg2.x += global_x;
        msg2.y += global_y;
        msg2.theta = global_theta + steer_theta;
    }
    map_free(map);
    std::cout << "AMCL TEST MISC" << std::endl;
    return 0;
}