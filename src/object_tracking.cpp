#include "object_tracking.h"

Category_Of_Points::Category_Of_Points(sensor_msgs::PointCloud point_cloud,int name)
{
    point_cloud_.header.frame_id = point_cloud.header.frame_id;
    point_cloud_.points.resize(point_cloud.points.size());
    for(int i = 0;i < point_cloud.points.size();i++)
    {
        point_cloud_.points[i].x = point_cloud.points[i].x;
        point_cloud_.points[i].y = point_cloud.points[i].y;
        point_cloud_.points[i].z = point_cloud.points[i].z;
    }
    name_ = name;
}

Category_Of_Points::~Category_Of_Points()
{}

void Category_Of_Points::setName(int name)
{
    name_ = name;
}

void Category_Of_Points::setVelocity(geometry_msgs::Point velocity)
{
    linear_velocity_.x = velocity.x;
    linear_velocity_.y = velocity.y;
    linear_velocity_.z = velocity.z;
}

///////////////////////////////////////////////----------------------------////////////////////////////////////////

Object_Tracking::Object_Tracking()
:cluster_threshold_slope_(0.15)
{
    dynamic_reconfigure::Server<point_cloud_test::point_cloud_processe_Config> server;
    dynamic_reconfigure::Server<point_cloud_test::point_cloud_processe_Config>::CallbackType f;
    f = boost::bind(&Object_Tracking::dynamicCb,this,_1,_2);
    server.setCallback(f);
    sub_points_ = n_.subscribe("/object_point_cloud",1,&Object_Tracking::subPointsCb,this);
    pub_vel_ = n_.advertise<geometry_msgs::PoseArray>("/object_velocity",1);
    pub_rviz_box_ = n_.advertise<visualization_msgs::MarkerArray>("/point_category",1);

    ros::spin();
}

Object_Tracking::~Object_Tracking()
{}

void Object_Tracking::pointCloudClassification(sensor_msgs::PointCloud point_cloud)
{
    if(point_cloud_.size() == 0)
    {
        return;
    }
    cout << "begin to classificate the point cloud." << endl;
    if(category_of_points_of_time_.size() >= 10)
    {
        for(int i = 0;i < category_of_points_of_time_[0].size();i++)
        {
            delete category_of_points_of_time_[0][i];
        }
        category_of_points_of_time_.erase(category_of_points_of_time_.begin(),category_of_points_of_time_.begin() + 1);
    }

    vector<Category_Of_Points*> new_category;
    while((point_cloud_.size() != 0) && ros::ok())
    {
        sensor_msgs::PointCloud templant_points;
        templant_points.header.frame_id = point_cloud.header.frame_id;
        for(int i = 0;i < point_cloud_.size();i++)
        {
            if(templant_points.points.size() == 0)
            {
                geometry_msgs::Point32 point1;
                point1.x = point_cloud_[i].x;
                point1.y = point_cloud_[i].y;
                point1.z = point_cloud_[i].z;
                templant_points.points.push_back(point1);
                point_cloud_.erase(point_cloud_.begin() + i,point_cloud_.begin() + i + 1);
                i--;
                continue;
            }

            if(minDistanceOfPointToCloud(point_cloud_[i],templant_points) < cluster_threshold_slope_ * distancePointToLidar(point_cloud_[i]))
            //if(minDistanceOfPointToCloud(point_cloud_[i],templant_points) < 0.2)
            {
                geometry_msgs::Point32 point1;
                point1.x = point_cloud_[i].x;
                point1.y = point_cloud_[i].y;
                point1.z = point_cloud_[i].z;
                templant_points.points.push_back(point1);
                point_cloud_.erase(point_cloud_.begin() + i,point_cloud_.begin() + i + 1);
                i--;
            }
        }

        new_category.push_back(new Category_Of_Points(templant_points,new_category.size()));
    }
    category_of_points_of_time_.push_back(new_category);

    displayInRviz();
}

double Object_Tracking::minDistanceOfPointToCloud(geometry_msgs::Point32 point,sensor_msgs::PointCloud cloud)
{
    double ans = 100000;
    int n;
    for(int i = 0;i < cloud.points.size();i++)
    {
        double distance = sqrt((point.x - cloud.points[i].x) * (point.x - cloud.points[i].x) 
                             + (point.y - cloud.points[i].y) * (point.y - cloud.points[i].y) 
                             + (point.z - cloud.points[i].z) * (point.z - cloud.points[i].z));
        if(ans > distance)
        {
            ans = distance;
            n = i;
        }
    }

    cout << cloud.points[n].x << "," << cloud.points[n].y << "," << cloud.points[n].z << endl
         << point.x << "," << point.y << "," << point.z << endl
         << ans << endl << endl;
    return ans;
}

double Object_Tracking::distancePointToLidar(geometry_msgs::Point32 point)
{
    double ans = 0;
    ans = sqrt((point.x - point_lidar_.x) * (point.x - point_lidar_.x) 
             + (point.y - point_lidar_.y) * (point.y - point_lidar_.y) 
             + (point.z - point_lidar_.z) * (point.z - point_lidar_.z));
    
    // cout << point_lidar_.x << "," << point_lidar_.y << "," << point_lidar_.z << endl
    //      << point.x << "," << point.y << "," << point.z << endl
    //      << ans << endl;
    return ans;
}

void Object_Tracking::displayInRviz()
{
    // geometry_msgs::PoseArray pose_array;
    // pose_array.header.frame_id = category_of_points_of_time_[0][0]->point_cloud_.header.frame_id;
    // int n = category_of_points_of_time_.size() - 1;
    // for(int i = 0;i < category_of_points_of_time_[n].size();i++)
    // {
    //     for(int j = 0;j < category_of_points_of_time_[n][i]->point_cloud_.points.size();j++)
    //     {
    //         geometry_msgs::Pose pose1;
    //         pose1.position.x = category_of_points_of_time_[n][i]->point_cloud_.points[j].x;
    //         pose1.position.y = category_of_points_of_time_[n][i]->point_cloud_.points[j].y;
    //         pose1.orientation.w = cos(0.5 * i / 2);
    //         pose1.orientation.z = sin(0.5 * i / 2);
    //         pose_array.poses.push_back(pose1);
    //     }
    // }
    // pub_vel_.publish(pose_array);

    visualization_msgs::MarkerArray boxs;
    visualization_msgs::Marker box1;
    box1.header.frame_id = "laser";
    box1.id = boxs.markers.size();
    box1.type = visualization_msgs::Marker::CUBE;
    box1.action = visualization_msgs::Marker::ADD;
    box1.pose.position.x = 0;
    box1.pose.position.y = 0;
    box1.pose.position.z = 0;
    box1.pose.orientation.x = 0.0;
    box1.pose.orientation.y = 0.0;
    box1.pose.orientation.z = 0.0;
    box1.pose.orientation.w = 1.0;
    box1.scale.x = 1.0;
    box1.scale.y = 1.0;
    box1.scale.z = 1.0;
    box1.color.r = 0.0f;
    box1.color.g = 1.0f;
    box1.color.b = 0.0f;
    box1.color.a = 1.0;
    boxs.markers.push_back(box1);
    pub_rviz_box_.publish(boxs);
}

void Object_Tracking::dynamicCb(point_cloud_test::point_cloud_processe_Config &config,uint32_t level)
{
    cout << "reconfigure request:" << config.cluster_threshold_slope << endl;
    cluster_threshold_slope_ = config.cluster_threshold_slope;
}

void Object_Tracking::subPointsCb(sensor_msgs::PointCloud msg)
{
    point_lidar_.x = msg.points[0].x;
    point_lidar_.y = msg.points[0].y;
    point_lidar_.z = msg.points[0].z;

    point_cloud_.clear();
    geometry_msgs::Point32 point1;
    for(int i = 1;i < msg.points.size();i++)
    {
        point1.x = msg.points[i].x;
        point1.y = msg.points[i].y;
        point1.z = msg.points[i].z;
        point_cloud_.push_back(point1);
    }
    pointCloudClassification(msg);
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "object_tracking");
    cout << "begin to object tracking." << endl;
    Object_Tracking test1;
    return 0;
}