#ifndef OBJECT_TRACKING_H
#define OBJECT_TRACKING_H

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <dynamic_reconfigure/server.h>
#include <point_cloud_test/point_cloud_processe_Config.h>
#include <visualization_msgs/MarkerArray.h>
#include "point_cloud_test/CategoryOfPoints.h"
#include <vector>
#include <string>

using namespace std;

class Category_Of_Points
{
public:
    sensor_msgs::PointCloud point_cloud_;
    int name_;
    geometry_msgs::Point linear_velocity_;

    Category_Of_Points();
    Category_Of_Points(sensor_msgs::PointCloud point_cloud,int name);
    ~Category_Of_Points();
    void setName(int name);
    void setVelocity(geometry_msgs::Point velocity);
};

using namespace std;

class Object_Tracking
{
public:
    Object_Tracking();
    ~Object_Tracking();

private:
    vector<vector<Category_Of_Points*> > category_of_points_of_time_;
    //vector<vector<point_cloud_test::CategoryOfPoints> > test_;
    vector<geometry_msgs::Point32> point_cloud_;
    geometry_msgs::Point32 point_lidar_;
    ros::NodeHandle n_;
    ros::Subscriber sub_points_;
    ros::Publisher pub_vel_;
    ros::Publisher pub_rviz_box_;
    void subPointsCb(sensor_msgs::PointCloud msg);

    double clusterRadio_; //d * cluster_threshold_slope
    double cluster_threshold_slope_;
    void pointCloudClassification(sensor_msgs::PointCloud point_cloud);
    double minDistanceOfPointToCloud(geometry_msgs::Point32 point,sensor_msgs::PointCloud cloud);
    double distancePointToLidar(geometry_msgs::Point32 point);

    void displayInRviz();

    vector<vector<double> > correlation_of_category_;
    double correlationOfFun();

    void dynamicCb(point_cloud_test::point_cloud_processe_Config &config,uint32_t level);
};

#endif