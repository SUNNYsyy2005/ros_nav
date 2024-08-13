#include <iostream>
#include "amcl/map/map.h"
#include "amcl/sensors/amcl_laser.h"z
#define M_PI 3.14159265358979323846
// 一个简单的随机姿态生成函数，用于初始化粒子
pf_vector_t random_pose_init(void *data) {
    pf_vector_t pose;
    // 这里假设一个均匀分布，实际应用中可能需要根据实际情况调整
    pose.v[0] = (double) rand() / RAND_MAX * 10.0 - 5.0; // x 坐标
    pose.v[1] = (double) rand() / RAND_MAX * 10.0 - 5.0; // y 坐标
    pose.v[2] = (double) rand() / RAND_MAX * 2 * M_PI - M_PI; // 角度，从 -π 到 π
    return pose;
}
int main(){
    map_t *map = map_alloc();
    map_load_occ(map, "map/1.pgm", 0.05,0);
    printf("%d %d %f %f\n", map->size_x, map->size_y, map->origin_x, map->origin_y);
    // 粒子滤波器的参数
    int min_samples = 50;      // 最小粒子数
    int max_samples = 1000;    // 最大粒子数
    double alpha_slow = 0.1;   // 慢衰减率
    double alpha_fast = 0.1;   // 快衰减率

    // 创建粒子滤波器
    pf_t *pf = pf_alloc(min_samples, max_samples, alpha_slow, alpha_fast,
                        random_pose_init, NULL);

    if (pf == NULL) {
        fprintf(stderr, "Failed to allocate particle filter\n");
        return -1;
    }
    printf("Particle filter created\n");
    // 初始均值和协方差矩阵
    // 这里我们假设一个初始的高斯分布，实际应用中可能需要根据实际情况调整
    pf_vector_t mean = {0, 0, 0}; // 初始均值 [x, y, theta]
    pf_matrix_t cov = {1.0, 0, 0, 0, 1.0, 0 , 0, 0, 0.1}; // 初始协方差

    // 使用高斯模型初始化粒子滤波器
    pf_init(pf, mean, cov);
    printf("Particle filter initialized\n");
    // 设置AMCL的激光雷达传感器模型
    amcl::AMCLLaser laser_sensor((size_t)30, map); // 假设最多处理30条激光束
    printf("Laser sensor created\n");
    laser_sensor.SetModelLikelihoodField(0.5, 0.5, 0.1, 1.0); // 激光模型参数
    printf("Laser sensor model set\n");
    // 假设我们有一个激光雷达数据
    amcl::AMCLLaserData laser_data;
    laser_data.sensor = &laser_sensor;
    laser_data.range_count = 360; // 假设有360个测量点
    laser_data.range_max = 10.0; // 最大测量距离
    laser_data.ranges.resize(360, std::vector<double>(2)); // 为360个测量点分配空间，每个点有2个值（距离和角度）
    printf("Laser data created\n");
    // 填充激光雷达数据
    for (int i = 0; i < 360; i++) {
        laser_data.ranges[i][0] = 5.0 - i * 0.1; // 假设的距离
        laser_data.ranges[i][1] = i * (2 * M_PI / 360); // 角度
    }
    printf("Laser data filled\n");
    // 将激光雷达数据更新到AMCL
    laser_sensor.UpdateSensor(pf, &laser_data);
    printf("Laser sensor data updated\n");
    // 获取AMCL的预测位置
    pf_vector_t predicted_pose;
    double total_weight = 0.0;
    for (int i = 0; i < pf->sets[pf->current_set].sample_count; i++) {
        pf_sample_t *sample = &(pf->sets[pf->current_set].samples[i]);
        predicted_pose.v[0] += sample->pose.v[0] * sample->weight;
        predicted_pose.v[1] += sample->pose.v[1] * sample->weight;
        predicted_pose.v[2] += sample->pose.v[2] * sample->weight;
        total_weight += sample->weight;
    }

    // 归一化预测位置
    if (total_weight > 0) {
        predicted_pose.v[0] /= total_weight;
        predicted_pose.v[1] /= total_weight;
        predicted_pose.v[2] /= total_weight;
    }

    // 打印预测位置
    std::cout << "Predicted pose (x, y, theta): ("
              << predicted_pose.v[0] << ", "
              << predicted_pose.v[1] << ", "
              << predicted_pose.v[2] << ")" << std::endl;

    map_free(map);
    std::cout << "AMCL TEST MISC" << std::endl;
    return 0;
}