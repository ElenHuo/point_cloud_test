#ifndef OBJECT_TRACKING_H
#define OBJECT_TRACKING_H

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32.h>
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
    int last_name_;
    int heredity_name_;
    double tx_;
    double centrol_x_;
    double centrol_y_;
    double centrol_z_;
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
    ros::Publisher pub_rqt_plot_;
    ros::Publisher pub_points_state_;
    void subPointsCb(sensor_msgs::PointCloud msg);
    void pubPointsState();
    void rqtPlotDisplay(float data);

    double clusterRadio_; //d * cluster_threshold_slope
    double cluster_threshold_slope_;
    void pointCloudClassification(sensor_msgs::PointCloud point_cloud);
    double minDistanceOfPointToCloud(geometry_msgs::Point32 point,sensor_msgs::PointCloud cloud);
    double distancePointToLidar(geometry_msgs::Point32 point);

    void displayInRviz();

    vector<vector<double> > correlation_of_category_;
    double w1_;
    double w2_;
    double var_of_points_bias_;
    double var_of_distance_bias_;
    void correlationOperate();
    double correlationOfFun(int now,int last,int period);
    double fun1(int now,int last,int period);
    double fun2(int now,int last,int period);
    int nameIsUsed(int name,int period);
    void connectWithLastName();
    double predicted_variance_;
    double measure_variance_;
    void estimateTheVelocity();
    void regressionAnalysis(geometry_msgs::Point& predicted_value_vel,vector<double> v_x,vector<double> v_y,vector<double> tt);
    void regressionAnalysis2(geometry_msgs::Point& predicted_value_vel,vector<double> v_x,vector<double> v_y,vector<double> tt);

    void dynamicCb(point_cloud_test::point_cloud_processe_Config &config,uint32_t level);
};

#endif