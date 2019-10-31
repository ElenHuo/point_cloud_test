#ifndef OWN_COST_MAP_H
#define OWN_COST_MAP_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <dynamic_reconfigure/server.h>
#include <point_cloud_test/point_cloud_processe_Config.h>
#include <string>
#include <pthread.h>
#include <signal.h>
#include "tf_listerner.h"

using namespace std;

class Own_Cost_Map
{
public:
    Own_Cost_Map();
    ~Own_Cost_Map();

private:
    ros::NodeHandle n_;
    ros::Subscriber sub_map_;
    ros::Subscriber sub_point_;
    ros::Subscriber sub_points_state_;
    ros::Publisher pub_own_cost_map_;
    ros::Publisher pub_dwa_map_;
    nav_msgs::OccupancyGrid origin_map_;
    nav_msgs::OccupancyGrid new_map_;
    int load_map_flag_;
    geometry_msgs::PoseArray origin_points_;
    //Tf_Listerner* car_in_map_;

    double inflation_radios_;
    double position_offset_;

    void dynamicCb(point_cloud_test::point_cloud_processe_Config &config,uint32_t level);
    void mapCb(nav_msgs::OccupancyGrid map);
    void pointsStateCb(geometry_msgs::PoseArray msg);
    void addInflationLayerToOriginMap();
    double distanceOffset(double distance,double car_vel_x,double car_vel_y,double point_vel_x,double point_vel_y);
    void pubDwaMap(nav_msgs::OccupancyGrid map);
    void pointCb(sensor_msgs::PointCloud msg);

    ros::Publisher pub_a_pose_;
    void mapTestDisplay(nav_msgs::OccupancyGrid map);

    void pthreadTwo();
    static void *threadTwo(void * arg);
    void threadRunTwo();
    pthread_t m_tid_;
};
#endif