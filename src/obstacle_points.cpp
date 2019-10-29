#include "obstacle_points.h"

Obstacle_Tracking::Obstacle_Tracking()
:search_scope_(20),load_map_flag_(0)
{
    tf_listerner1 = new Tf_Listerner("map","laser");

    sub_map_ = n_.subscribe("/map",2,&Obstacle_Tracking::mapCb,this);
    while(load_map_flag_ == 0) 
    {
        ros::spinOnce();
    }
    dynamic_reconfigure::Server<point_cloud_test::point_cloud_processe_Config> server;
    dynamic_reconfigure::Server<point_cloud_test::point_cloud_processe_Config>::CallbackType f;
    f = boost::bind(&Obstacle_Tracking::dynamicCb,this,_1,_2);
    server.setCallback(f);
    sub_laser_ = n_.subscribe("/scan", 2, &Obstacle_Tracking::laserCb,this);
    pub_point_cloud_ = n_.advertise<sensor_msgs::PointCloud>("/object_point_cloud", 1);
    ros::spin();
}

Obstacle_Tracking::~Obstacle_Tracking()
{}

void Obstacle_Tracking::dynamicCb(point_cloud_test::point_cloud_processe_Config &config,uint32_t level)
{
    cout << "reconfigure request:" << config.inflation_radius << endl;
    search_scope_ = config.inflation_radius / map_.info.resolution;
}

void Obstacle_Tracking::laserCb(sensor_msgs::LaserScan msg)
{
    if(load_map_flag_ == 0) return;

    tf::Quaternion q1;
    tf::Matrix3x3 M1;
    sensor_msgs::PointCloud point_cloud;
    //point_cloud.header.frame_id = "laser";
    point_cloud.header.frame_id = "map";
    geometry_msgs::Point32 point1;
    point1.x = tf_listerner1->x();
    point1.y = tf_listerner1->y();
    point1.z = tf_listerner1->z();
    point_cloud.points.push_back(point1);
    for(int i = 0;i < msg.ranges.size();i++)
    {
        q1[0] = 0;
        q1[1] = 0;
        q1[2] = tf_listerner1->oz();
        q1[3] = tf_listerner1->ow();
        M1.setRotation(q1);
        if((msg.ranges[i] > msg.range_min) && (msg.ranges[i] < msg.range_max))
        {
            point1.x = msg.ranges[i] * cos(msg.angle_min + i * msg.angle_increment);
            point1.y = msg.ranges[i] * sin(msg.angle_min + i * msg.angle_increment);
            double world_x = M1[0][0] * point1.x + M1[0][1] * point1.y + tf_listerner1->x() - map_.info.origin.position.x;
            double world_y = M1[1][0] * point1.x + M1[1][1] * point1.y + tf_listerner1->y() - map_.info.origin.position.y;
            int map_data_n = (int)(world_y / map_.info.resolution) * map_.info.width + world_x / map_.info.resolution;
            if((map_.data[map_data_n] == 0) && (distanceToObstacle(world_x,world_y) == 100))
            {
                point1.x = world_x + map_.info.origin.position.x;
                point1.y = world_y + map_.info.origin.position.y;
                point_cloud.points.push_back(point1);
            }
        }
    }
    pub_point_cloud_.publish(point_cloud);

    static double last_time = 0;
    double now_time = ros::Time::now().toSec();
    if(now_time - last_time < 0.1) ros::Duration(0.1 + last_time - now_time).sleep();
    last_time = ros::Time::now().toSec();
}

void Obstacle_Tracking::mapCb(nav_msgs::OccupancyGrid map)
{
    map_.header.stamp = map.header.stamp;
    map_.header.frame_id = map.header.frame_id;
    map_.info.map_load_time = map.info.map_load_time;
    map_.info.resolution = map.info.resolution;
    map_.info.width = map.info.width;
    map_.info.height = map.info.height;
    map_.info.origin.position.x = map.info.origin.position.x;
    map_.info.origin.position.y = map.info.origin.position.y;
    map_.info.origin.position.z = map.info.origin.position.z;
    map_.info.origin.orientation.x = map.info.origin.orientation.x;
    map_.info.origin.orientation.y = map.info.origin.orientation.y;
    map_.info.origin.orientation.z = map.info.origin.orientation.z;
    map_.info.origin.orientation.w = map.info.origin.orientation.w;
    map_.data.resize(map.data.size());
    for(int i = 0;i < map.data.size();i++)
    {
        map_.data[i] = map.data[i];
    }
    cout << "load the map successed" << endl;
    load_map_flag_ = 1;
}

double Obstacle_Tracking::distanceToObstacle(double x,double y)
{
    int width = x / map_.info.resolution;
    int height = y / map_.info.resolution;
    int obs_x;
    int obs_y;
    for(int i = 0;i < search_scope_;i++)
    {
        for(int j = 0 - i;j < i + 1;j++)
        {
            if((width + j >= 0) && (width + j < map_.info.width) && (height + i >= 0) && (height + i < map_.info.height))
            {
                if(map_.data[(height + i) * map_.info.width + width + j] == 100)
                {
                    obs_x = width + j;
                    obs_y = height + i;
                    return sqrt((obs_x * map_.info.resolution - x) * (obs_x * map_.info.resolution - x) + (obs_y * map_.info.resolution - y) * (obs_y * map_.info.resolution - y));
                    break;
                }
            }
        }
        for(int j = 0 - i;j < i + 1;j++)
        {
            if((width + j >= 0) && (width + j < map_.info.width) && (height - i >= 0) && (height - i < map_.info.height))
            {
                if(map_.data[(height - i) * map_.info.width + width + j] == 100)
                {
                    obs_x = width + j;
                    obs_y = height - i;
                    return sqrt((obs_x * map_.info.resolution - x) * (obs_x * map_.info.resolution - x) + (obs_y * map_.info.resolution - y) * (obs_y * map_.info.resolution - y));
                    break;
                }
            }
        }
        for(int j = 0 - i;j < i + 1;j++)
        {
            if((width + i >= 0) && (width + i < map_.info.width) && (height + j >= 0) && (height + j < map_.info.height))
            {
                if(map_.data[(height + j) * map_.info.width + width + i] == 100)
                {
                    obs_x = width + i;
                    obs_y = height + j;
                    return sqrt((obs_x * map_.info.resolution - x) * (obs_x * map_.info.resolution - x) + (obs_y * map_.info.resolution - y) * (obs_y * map_.info.resolution - y));
                    break;
                }
            }
        }
        for(int j = 0 - i;j < i + 1;j++)
        {
            if((width - i >= 0) && (width - i < map_.info.width) && (height + j >= 0) && (height + j < map_.info.height))
            {
                if(map_.data[(height + j) * map_.info.width + width + i] == 100)
                {
                    obs_x = width - i;
                    obs_y = height + j;
                    return sqrt((obs_x * map_.info.resolution - x) * (obs_x * map_.info.resolution - x) + (obs_y * map_.info.resolution - y) * (obs_y * map_.info.resolution - y));
                    break;
                }
            }
        }
    }
    return 100;
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "obstacle_points");
    cout << "begin obstacle tracking node." << endl;

    Obstacle_Tracking test1;
    
    return 0;
}
