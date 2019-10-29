#ifndef ASTAR_H
#define ASTAR_H

#include<iostream>
#include<ros/ros.h>
#include<dynamic_reconfigure/server.h>
#include<vector>
#include<list>
#include<string>
#include "tf_listerner.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/PointCloud.h"

using namespace std;

class Node{
public: 
    int x;//r  行
    int y;//c  列
    int state; //100 obstacle 50 inflation  -1 unknown 0 free 
    int num;
    double G;
    double H;
    double F;
    int visit; //close list
    int open_visit; //openlist
    Node* parentnode;
    double path_x;
    double path_y;
    Node()
    {
        x = 0;
        y = 0;
        state = 0;
        num = 0;
        G = 0;
        H = 0;
        F = 0;
        visit = 0;
        open_visit = 0;
    }
};


using namespace std;
class Astar
{
public:



    Astar();//making function
    ~Astar();

private:
    int load_map_flag_;
    int goal_flag;
    int width;
    int height;
    Tf_Listerner* car_in_map_;

    Node car_pos;
    Node goal;
  

    vector<Node> node;
    list<Node*> openlist;
    vector<Node> path;
    vector<Node> temp_path;

    nav_msgs::OccupancyGrid map_;  
    geometry_msgs::Pose goal_;
    nav_msgs::Path global_path;
    ///////////////////test///////
    sensor_msgs::PointCloud PC_TEST;
    //////////////////teste////////

    ros::NodeHandle n;
    ros::Subscriber sub_costmap;
    ros::Subscriber sub_goal;
    ros::Publisher  pub_path;
    ros::Publisher  PC_pub;

    void sub_costmapCB(nav_msgs::OccupancyGrid costmap);
    void sub_goalCB(geometry_msgs::Pose goal);
    void Algorithm();
    void Goal_Convert();
    double Manhattandistance(Node A,Node B);
    double Linedistance (Node A, Node B);
    void F_value_cal(Node* Current , Node* goal);
    int obstacle_check_Start2Goal();
    void Astar_Search_Algorithm();  
    void Path_Smooth();
    int Line_Obstacle_Check(Node A ,Node B);
    int minInOpenList();
    int CheckPath();    
    


    void PthreadOne();
    static void *ThreadOne(void * arg);
    void ThreadRunOne();
    pthread_t m_tid;
};
















#endif