#ifndef OBSTACLE_TRACKING_H
#define OBSTACLE_TRACKING_H

#include <iostream>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <point_cloud_test/point_cloud_processe_Config.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include "nav_msgs/OccupancyGrid.h"
#include <vector>
#include "tf_listerner.h"

using namespace std;

class Obstacle_Tracking
{
public:
    Obstacle_Tracking();
    ~Obstacle_Tracking();

private:
    nav_msgs::OccupancyGrid map_;
    ros::NodeHandle n_;
    ros::Subscriber sub_laser_;
    ros::Subscriber sub_map_;
    ros::Publisher pub_point_cloud_;

    Tf_Listerner* tf_listerner1;
    
    int load_map_flag_;
    void laserCb(sensor_msgs::LaserScan msg);
    void mapCb(nav_msgs::OccupancyGrid map);
    void dynamicCb(point_cloud_test::point_cloud_processe_Config &config,uint32_t level);

    int search_scope_;
    double distanceToObstacle(double x,double y);
};

//void dynamicCb(point_cloud_test::point_cloud_processe_Config &config,uint32_t level);

#endif