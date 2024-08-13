#include <iostream>
#include "inc/teb_config.h"
#include "inc/pose_se2.h"
#include "inc/robot_footprint_model.h"
#include "inc/obstacles.h"
#include "inc/optimal_planner.h"
#include <boost/smart_ptr.hpp>
//
#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>
#include <ros/ros.h>
#include "src/action.h"
using namespace teb_local_planner;

int main(int argc, char** argv)
{
    // 参数配置
    TebConfig config;
    PoseSE2 start(100, 0, 0);
    PoseSE2 end(300, 200, 0);
    std::vector<ObstaclePtr> obst_vector;
    obst_vector.emplace_back(boost::make_shared<PointObstacle>(200, 100));
    obst_vector.emplace_back(boost::make_shared<PointObstacle>(300, 100));
    ViaPointContainer via_points;
    // Setup robot shape model
    RobotFootprintModelPtr robot_model = boost::make_shared<CircularRobotFootprint>(0.4);
    auto visual = TebVisualizationPtr(new TebVisualization(config));
    auto planner = new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points);
    cv::Mat show_map = cv::Mat::zeros(cv::Size(500, 500), CV_8UC3);

    // param
    int start_theta = 0;
    int end_theta = 0;
    ros::init(argc, argv, "teb_node");
    Action action_module;
    ros::Rate rate(50); // 50Hz

    while (true)
    {
        memset(show_map.data, 0, 500 * 500 * 3);
        try
        {
            start.theta() = start_theta * 0.1;
            end.theta() = end_theta * 0.1;

            auto s = std::chrono::high_resolution_clock::now();

            planner->plan(start, end);
            // do somthine
            auto e = std::chrono::high_resolution_clock::now();
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(e - s);

            std::cout << "cost "<< ms.count()  <<" ms"<< std::endl;
            // vi
            std::vector<Eigen::Vector3f> path;
            planner->getFullTrajectory(path);
            printf("path size: %d\n", path.size());
            for (int i = 0; i < path.size() - 1; i++)
            {
                int x = (int)(path.at(i)[0]);
                int y = (int)(path.at(i)[1] );
                printf("x: %d, y: %d\n", x, y);
                int next_x = (int)(path.at(i + 1)[0]);
                int next_y = (int)(path.at(i + 1)[1]);
                cv::line(show_map, cv::Point(x, y), cv::Point(next_x, next_y), cv::Scalar(255, 255, 255));
            }
            cv::createTrackbar("start theta", "path", &start_theta, 100);
            cv::createTrackbar("end theta", "path", &end_theta, 100);
            cv::imshow("path", show_map);
        }
        catch (...)
        {
            break;
        }
        cv::waitKey(10);
    }
    return 0;
}
