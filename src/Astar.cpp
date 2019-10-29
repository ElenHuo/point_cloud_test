#include "Astar.h"

Astar::Astar()
:load_map_flag_(0),goal_flag(0)
{
    ////////////////////////////////////////////////////////////test///////////////
    geometry_msgs::Pose goal_test;
    sub_goalCB(goal_test);
    ////////////////////////////////////////////////////////////test///////////////
    //sub_costmap = n.subscribe("/own_cost_map", 1 ,&Astar::sub_costmapCB,this);
    sub_costmap = n.subscribe("/map", 1 ,&Astar::sub_costmapCB,this);////////////////////////////////////////////////test
    sub_goal = n.subscribe("/own_destination", 1 ,&Astar::sub_goalCB,this);
    pub_path = n.advertise<nav_msgs::Path>("/own_path",1);

    PC_pub =  n.advertise<sensor_msgs::PointCloud>("/PC_test",1);

    car_in_map_ = new Tf_Listerner("map","base_footprint");
    PthreadOne();

    ros::spin();//monitor CB function
}

Astar::~Astar()
{
    cout << "programme stop running!" <<endl;
}

void Astar::sub_costmapCB(nav_msgs::OccupancyGrid costmap)
{
    double t0 = ros::Time::now().toSec();
    map_ = costmap;
    width = map_.info.width;
    height = map_.info.height;
    node.clear();
    node.resize(costmap.data.size());

    for(int i = 0;i < costmap.data.size();i++)  //n convert to point[r,c]
    {
        node[i].x =  i  % map_.info.width ; //in WORLD COORDINATE [R,C]
        node[i].y =  i / map_.info.width ;
        node[i].num = i;
        node[i].state = costmap.data[i];

    }

    tf::Quaternion q2;
    tf::Matrix3x3 M2;
    q2[0] = car_in_map_->ox();
    q2[1] = car_in_map_->oy();
    q2[2] = car_in_map_->oz();
    q2[3] = car_in_map_->ow();
    M2.setRotation(q2); //origin shows in map coordinate

    double car_pos_x = car_in_map_->x()  - map_.info.origin.position.x;
    double car_pos_y = car_in_map_->y() - map_.info.origin.position.y;

    //////////////////test///////////
     car_pos_x = -6 - map_.info.origin.position.x;
     car_pos_y = -10 - map_.info.origin.position.y;
    //////////////////teste///////////
    car_pos.num = ( (int)( car_pos_y / map_.info.resolution) * map_.info.width + car_pos_x / map_.info.resolution);

    //car_pos of [r,c]
    car_pos.x = car_pos.num % width;  //储存为行
    car_pos.y = car_pos.num / width;  //储存为列

    cout<<"CAR_POS:"<<car_pos.x<<","<<car_pos.y<<" Num:"<<car_pos.num<<endl;

    for(int i = 0;i < node.size();i++)  //n convert to point[r,c]
    {
        if (node[i].num ==car_pos.num)
        {
            car_pos.state =node[i].state;
        }
    }
    cout << "Start_point.state:  "<<car_pos.state<<endl;
    cout << "load the map successed" << endl;
    load_map_flag_ = 1;
    Goal_Convert();// goal to world coordinate
    cout << "the time of load the map is:" << ros::Time::now().toSec() - t0 << endl;
    Algorithm();
}

void Astar::sub_goalCB(geometry_msgs::Pose goal)
{
    goal_ = goal;
    ///////////////////////////////////////////////////////////////////test
    goal_.position.x = 2;
    goal_.position.y = 1;
    ///////////////////////////////////////////////////////////////////test
    goal_flag = 1;

}

void Astar::Goal_Convert()
{
    if(goal_flag == 1)
    {
        tf::Quaternion q1;
        tf::Matrix3x3 M1;
        /*
        q1[0] = map_.info.origin.orientation.x;
        q1[1] = map_.info.origin.orientation.y;
        q1[2] = map_.info.origin.orientation.z;
        q1[3] = map_.info.origin.orientation.w;
        */
        M1.setRotation(q1); //origin shows in map coordinate
        double goal_x = goal_.position.x - map_.info.origin.position.x;
        double goal_y = goal_.position.y - map_.info.origin.position.y;

        goal.num = (  (int)(goal_y / map_.info.resolution) * map_.info.width + goal_x / map_.info.resolution);


        for(int i = 0;i < node.size();i++)  //n convert to point[r,c]
        {

        if (node[i].num ==goal.num)
            goal.state =node[i].state ;
        }
        //goal [r,c] convert
        goal.x = goal.num % width;  //储存为行
        goal.y = goal.num / width;  //储存为列
        cout<<"NUM:" << goal.num<<"   X,Y,STATE:"<< goal.x<<" "<<goal.y<<", "<<goal.state<< endl;

        if(goal.state != 0  ) //obstacle or unknow area
        {
            cout<<"The destination is unacceptable to arrive!"<<endl;
        }
        else cout<<"Goal achieved"<<endl;
    }
}

double Astar::Manhattandistance(Node A,Node B)
{
    return    (abs(A.x-B.x)+abs(A.y-B.y)); //Manhattan Dis
    
}
double Astar::Linedistance (Node A, Node B)
{
    return sqrt (  (A.x-B.x)*(A.x-B.x)+(A.y-B.y)*(A.y-B.y)  ) ;
}

void Astar::F_value_cal(Node* Current , Node* Goal)
{

    //Current->G = Current->parentnode->G + Linedistance(*Current,*(Current->parentnode));
    
    Current->G = Current->parentnode->G + Linedistance(*Current,*(Current->parentnode));
    Current->H = 1.5 *Linedistance(*Current, *Goal);
    Current->F = Current->G + Current->H;

}

int Astar::obstacle_check_Start2Goal()
{
    

    int check_y;
    if(car_pos.x == goal.x)
    {
        if(goal.y>car_pos.y)
        {
            for (int i = car_pos.y+1; i < goal.y ; i++ )
            {
                if( ( node[i*width + goal.x].state == -1) || 
                ( node[i*width + goal.x].state > 30 ) )//num 有障碍物或者未知区域
                {
                    return 1 ;
                }
            }
            //离散点的个数
            int point_num = (goal.y - car_pos.y) *  map_.info.resolution /  0.15  ;  //1m = 5points
            path.push_back(car_pos);
            for(int i = 1; i<=point_num;i++)
            {
                int point_ = car_pos.y + i * (goal.y - car_pos.y) / point_num; 
                Node _point = node[ point_ * width + car_pos.x ];
                path.push_back(_point);
            }


        }
        else
        {
            for (int i = goal.y+1; i < car_pos.y ; i++ )
            {
                if( ( node[i*width + goal.x].state == -1) || 
                ( node[i*width  + goal.x].state > 30 ) )//num 有障碍物或者未知区域
                {
                    return 1 ;
                }
            }
            //离散点的个数
            int point_num = (car_pos.y - goal.y) *  map_.info.resolution /  0.15  ;
            path.push_back(car_pos);
            for(int i = 1; i<=point_num;i++)
            {
                int point_ = car_pos.y - i * (car_pos.y - goal.y) / point_num; 
                Node _point = node[ point_ * width + car_pos.x ];
                path.push_back(_point);
            }            
        }
        
    }
    else
    {
        double K =  (car_pos.y - goal.y) / (car_pos.x - goal.x);
        double B =  (goal.y - K * goal.x);
        if( K >= 0 )
        {
            if(goal.x > car_pos.x)    //对x进行插补
            {
                
                for (int i = car_pos.x+1; i < goal.x ; i++ )
                {
                    check_y = K * i  + B ;
                    if( ( node[(check_y * width + i)].state == -1) || 
                        ( node[(check_y * width + i)].state > 30 ) )//num 有障碍物或者未知区域
                        {
                            return 1 ;
                        }
                }  
                for (int i = car_pos.y+1; i < goal.y ; i++ ) //对Y
                {
                    check_y =  (i - B) /K ;
                    if( ( node[(i * width + check_y)].state == -1) || 
                        ( node[(i * width + check_y)].state > 30 ) )//num 有障碍物或者未知区域
                        {
                            return 1 ;
                        }
                }
                //离散点的个数/////////////////////////////////////
                if(K>1)//在Y方向插值
                {
                    int point_num = (goal.y - car_pos.y) *  map_.info.resolution /  0.15  ;
                    path.push_back(car_pos);
                    for(int i = 1; i<=point_num;i++)
                    {
                        int point_ = car_pos.y + i * (goal.y - car_pos.y) / point_num; 
                        int x = (point_ - B) / K ;
                        Node _point = node[ point_ * width + x ];
                        path.push_back(_point);
                    }   
                }
                else 
                {
                    int point_num = (goal.x - car_pos.x) *  map_.info.resolution /  0.15  ;
                    path.push_back(car_pos);
                    for(int i = 1; i<=point_num;i++)
                    {
                        int x = car_pos.x + i * (goal.x - car_pos.x) / point_num; 
                        int point_ = K * x + B;
                        
                        Node _point = node[ point_ * width + x ];
                        path.push_back(_point);
                    }    
                }


            }
            else if(goal.x < car_pos.x)
            {
                for (int i = goal.x+1; i < car_pos.x ; i++ )  //对X\Y都进行一次遍历搜索
                {
                    check_y = K * i  + B ;
                    if( ( node[(check_y * width + i)].state == -1) || 
                        ( node[(check_y * width + i)].state > 30 ) )//num 有障碍物或者未知区域
                        {
                            return 1 ;
                        }
                }
                for (int i = goal.y+1; i < car_pos.y ; i++ )
                {
                    check_y =  (i - B) /K ;
                    if( ( node[(i * width + check_y)].state == -1) || 
                        ( node[(i * width + check_y)].state > 30 ) )//num 有障碍物或者未知区域
                        {
                            return 1 ;
                        }
                } 
               //离散点的个数/////////////////////////////////////
                if(K>1)//在Y方向插值
                {
                    int point_num = (car_pos.y - goal.y) *  map_.info.resolution /  0.15  ;
                    path.push_back(car_pos);
                    for(int i = 1; i<=point_num;i++)
                    {
                        int point_ = car_pos.y - i * (car_pos.y - goal.y) / point_num; 
                        int x = (point_ - B) / K ;
                        Node _point = node[ point_ * width + x ];
                        path.push_back(_point);
                    }   
                }
                else 
                {
                    int point_num = (car_pos.x - goal.x) *  map_.info.resolution /  0.15  ;
                    path.push_back(car_pos);
                    for(int i = 1; i<=point_num;i++)
                    {
                        int x = car_pos.x - i * (car_pos.x - goal.x) / point_num; 
                        int point_ = K * x + B;
                        
                        Node _point = node[ point_ * width + x ];
                        path.push_back(_point);
                    }    
                }
            }
        }
        else if(K < 0)
        {
            if(goal.x > car_pos.x)    //对x进行插补
            {
                
                for (int i = car_pos.x+1; i < goal.x ; i++ )
                {
                    check_y = K * i  + B ;
                    if( ( node[(check_y * width + i)].state == -1) || 
                        ( node[(check_y * width + i)].state > 30 ) )//num 有障碍物或者未知区域
                        {
                            return 1 ;
                        }
                }  
                for (int i = goal.y+1; i < car_pos.y ; i++ )
                {
                    check_y =  (i - B) /K ;
                    if( ( node[(i * width + check_y)].state == -1) || 
                        ( node[(i * width + check_y)].state > 30 ) )//num 有障碍物或者未知区域
                        {
                            return 1 ;
                        }
                }  
                //离散点的个数/////////////////////////////////////
                if(K < -1)//在Y方向插值
                {
                    int point_num = (car_pos.y - goal.y) *  map_.info.resolution /  0.15  ;
                    path.push_back(car_pos);
                    for(int i = 1; i<=point_num;i++)
                    {
                        int point_ = car_pos.y - i * (car_pos.y - goal.y) / point_num; 
                        int x = (point_ - B) / K ;
                        Node _point = node[ point_ * width + x ];
                        path.push_back(_point);
                    }   
                }
                else 
                {
                    int point_num = (goal.x - car_pos.x) *  map_.info.resolution /  0.15  ;
                    path.push_back(car_pos);
                    for(int i = 1; i<=point_num;i++)
                    {
                        int x = car_pos.x + i * (goal.x - car_pos.x) / point_num; 
                        int point_ = K * x + B;
                        
                        Node _point = node[ point_ * width + x ];
                        path.push_back(_point);
                    }    
                }

            }

            else if(goal.x < car_pos.x)
            {
                for (int i = goal.x+1; i < car_pos.x ; i++ )
                {
                    check_y = K * i  + B ;
                    if( ( node[(check_y * width + i)].state == -1) || 
                        ( node[(check_y * width + i)].state > 30 ) )//num 有障碍物或者未知区域
                        {
                            return 1 ;
                        }
                }
                for (int i = car_pos.y+1; i < goal.y ; i++ )
                {
                    check_y =  (i - B) /K ;
                    if( ( node[(i * width + check_y)].state == -1) || 
                        ( node[(i * width + check_y)].state > 30 ) )//num 有障碍物或者未知区域
                        {
                            return 1 ;
                        }
                } 
                    
                 //离散点的个数/////////////////////////////////////
                if(K < -1)//在Y方向插值
                {
                    int point_num = (goal.y - car_pos.y) *  map_.info.resolution /  0.15  ;
                    path.push_back(car_pos);
                    for(int i = 1; i<=point_num;i++)
                    {
                        int point_ = car_pos.y + i * (goal.y - car_pos.y) / point_num; 
                        int x = (point_ - B) / K ;
                        Node _point = node[ point_ * width + x ];
                        path.push_back(_point);
                    }   
                }
                else 
                {
                    int point_num = (car_pos.x - goal.x) *  map_.info.resolution /  0.15  ;
                    path.push_back(car_pos);
                    for(int i = 1; i<=point_num;i++)
                    {
                        int x = car_pos.x - i * (car_pos.x - goal.x) / point_num; 
                        int point_ = K * x + B;
                        
                        Node _point = node[ point_ * width + x ];
                        path.push_back(_point);
                    }    
                }
            }

        }
    }

    return 0 ;
}

/*
      达到一定条件开始搜索  A*寻路算法
*/
void Astar::Astar_Search_Algorithm()  
{
    double t0 = ros::Time::now().toSec();

    while(openlist.size() != 0)
    {

        int min_n = minInOpenList();


        if(node[min_n].x-1 > 0)
        {
            if(node[min_n - 1].state >= 0 && node[min_n - 1].state < 30 && node[min_n - 1].visit != 1)
            {
                if(node[min_n -1 ].open_visit !=1 )
                {

                    node[min_n - 1].parentnode = &node[min_n];
                    node[min_n - 1].open_visit = 1;
                    F_value_cal(&node[min_n - 1] , &goal);
                    openlist.push_back(&node[min_n - 1]);

                }
                else if (node[min_n -1].open_visit == 1 )
                {

                    if( ( node[min_n].G + Linedistance( node[min_n],node[min_n - 1]) )
                    < node[min_n -1].G)
                    {
                        node[min_n - 1].parentnode = &node[min_n];
                        F_value_cal(&node[min_n - 1] , &goal);
                    }

                }

            }


        }

        if(node[min_n].x + 1 < width)
        {
            if(node[min_n + 1].state >= 0 && node[min_n + 1].state < 30 && node[min_n + 1].visit!=1)
            {
                if(node[min_n + 1 ].open_visit !=1 )
                {

                    node[min_n + 1].parentnode = &node[min_n];
                    node[min_n + 1].open_visit = 1;
                    F_value_cal(&node[min_n + 1] , &goal);
                    openlist.push_back(&node[min_n + 1]);
                }
                else if (node[min_n + 1].open_visit == 1 )
                {

                    if( ( node[min_n].G + Linedistance( node[min_n],node[min_n + 1]) )
                    < node[min_n + 1].G)
                    {
                        node[min_n + 1].parentnode = &node[min_n];
                        F_value_cal(&node[min_n + 1] , &goal);
                    }

                }

            }
        }

        if(node[min_n].y + 1 <height)
        {
            if(node[min_n + width].state >= 0 && node[min_n + width].state < 30 && node[min_n + width].visit!=1)
            {
                if(node[min_n + width ].open_visit !=1 )
                {

                    node[min_n + width].parentnode = &node[min_n];
                    node[min_n + width].open_visit = 1;
                    F_value_cal(&node[min_n + width] , &goal);
                    openlist.push_back(&node[min_n + width]);
                }
                else if (node[min_n + width].open_visit == 1 )
                {

                    if( ( node[min_n].G + Linedistance( node[min_n],node[min_n + width]) )
                    < node[min_n + width].G)
                    {
                        node[min_n + width].parentnode = &node[min_n];
                        F_value_cal(&node[min_n + width] , &goal);
                    }

                }
            }

        }

        if(node[min_n].y - 1 > 0)
        {
            if(node[min_n - width].state >= 0 && node[min_n - width].state < 30 && node[min_n - width].visit!=1)
            {
                if(node[min_n - width ].open_visit != 1 )
                {

                    node[min_n - width].parentnode = &node[min_n];
                    node[min_n - width].open_visit = 1;
                    F_value_cal(&node[min_n - width] ,&goal);
                    openlist.push_back(&node[min_n - width]);

                }
                else if (node[min_n - width].open_visit == 1 )
                {

                    if( (node[min_n].G + Linedistance( node[min_n],node[min_n - width]) )
                    < node[min_n - width].G)
                    {
                        node[min_n - width].parentnode = &node[min_n];
                        F_value_cal(&node[min_n - width] , &goal);
                    }

                }
            }
        }

        /*                              对角判断                            */
        if(node[min_n].x - 1 > 0  && node[min_n].y + 1 <height )
        {
            if(node[min_n + width - 1 ].state >= 0 && node[min_n + width - 1 ].state < 30 && node[min_n + width - 1].visit!=1)
            {
                if(node[min_n + width - 1 ].open_visit != 1 )
                {

                    node[min_n + width - 1 ].parentnode = &node[min_n];
                    node[min_n + width - 1 ].open_visit = 1;
                    F_value_cal(&node[min_n + width - 1 ] ,&goal);
                    openlist.push_back(&node[min_n + width - 1 ]);

                }
                else if (node[min_n + width - 1 ].open_visit == 1 )
                {

                    if( (node[min_n].G + Linedistance( node[min_n],node[min_n + width - 1 ]) )
                    < node[min_n + width - 1 ].G)
                    {
                        node[min_n + width - 1 ].parentnode = &node[min_n];
                        F_value_cal(&node[min_n + width - 1] , &goal);
                    }

                }
            }
        }

        if(node[min_n].x + 1 < width  && node[min_n].y + 1 <height )
        {
            if(node[min_n + width + 1 ].state >= 0 && node[min_n + width + 1 ].state < 30 && node[min_n + width + 1].visit!=1)
            {
                if(node[min_n + width + 1 ].open_visit != 1 )
                {

                    node[min_n + width + 1 ].parentnode = &node[min_n];
                    node[min_n + width + 1 ].open_visit = 1;
                    F_value_cal(&node[min_n + width + 1 ] ,&goal);
                    openlist.push_back(&node[min_n + width + 1 ]);

                }
                else if (node[min_n + width + 1 ].open_visit == 1 )
                {

                    if( (node[min_n].G + Linedistance( node[min_n],node[min_n + width + 1 ]) )
                    < node[min_n + width + 1 ].G)
                    {
                        node[min_n + width + 1 ].parentnode = &node[min_n];
                        F_value_cal(&node[min_n + width + 1] , &goal);
                    }

                }
            }
        }       

        if(node[min_n].x - 1 > 0  && node[min_n].y - 1 > 0  )
        {
            if(node[min_n - width - 1 ].state >= 0 && node[min_n - width - 1 ].state < 30 && node[min_n - width - 1].visit!=1)
            {
                if(node[min_n - width - 1 ].open_visit != 1 )
                {

                    node[min_n - width - 1 ].parentnode = &node[min_n];
                    node[min_n - width - 1 ].open_visit = 1;
                    F_value_cal(&node[min_n - width - 1 ] ,&goal);
                    openlist.push_back(&node[min_n - width - 1 ]);

                }
                else if (node[min_n - width - 1 ].open_visit == 1 )
                {

                    if( (node[min_n].G + Linedistance( node[min_n],node[min_n - width - 1]) )
                    < node[min_n - width - 1 ].G)
                    {
                        node[min_n - width - 1 ].parentnode = &node[min_n];
                        F_value_cal(&node[min_n - width - 1] , &goal);
                    }

                }
            }
        }       

        if(node[min_n].x + 1 < width  && node[min_n].y - 1 >0  )
        {
            if(node[min_n - width + 1].state >= 0 && node[min_n - width + 1 ].state < 30 && node[min_n - width + 1].visit!=1)
            {
                if(node[min_n - width + 1 ].open_visit != 1 )
                {

                    node[min_n - width + 1 ].parentnode = &node[min_n];
                    node[min_n - width + 1 ].open_visit = 1;
                    F_value_cal(&node[min_n - width + 1 ] ,&goal);
                    openlist.push_back(&node[min_n - width + 1 ]);

                }
                else if (node[min_n - width + 1 ].open_visit == 1 )
                {

                    if( (node[min_n].G + Linedistance( node[min_n],node[min_n - width + 1 ]) )
                    < node[min_n - width + 1 ].G)
                    {
                        node[min_n - width + 1 ].parentnode = &node[min_n];
                        F_value_cal(&node[min_n - width + 1] , &goal);
                    }

                }
            }
        }   

        //goal is in the openlist
        if(node[goal.num].open_visit == 1)
        {
            cout << "the A* searching time is:" << ros::Time::now().toSec() - t0 << endl;
            break;
        }

    }
    //get parent node and make path
    if(node[goal.num].open_visit != 1) cout<<"Path is not found! Please check again!"<<endl;
    else if(node[goal.num].open_visit == 1)
    {
        Node ptr = node[goal.num];
        path.push_back(node[goal.num]);
        while( ptr.num != car_pos.num)
        {            
            path.push_back(*(ptr.parentnode));
            ptr = *(ptr.parentnode);
        }
        temp_path.clear(); //记得清空path点
        temp_path.resize(path.size());
        int count_num=0;
        geometry_msgs::PoseStamped position;
        for(int i= path.size()-1 ; i>=0 ; i--)
        {
            temp_path[count_num] = path[i];
            count_num++;

        }

        Path_Smooth();
        cout << "size:"<< temp_path.size()<< ","<< path.size() <<endl;
        // for(int i = 0;i < temp_path.size();i++)
        // {
        //     position.pose.position.x = temp_path[i].x * map_.info.resolution + map_.info.origin.position.x;
        //     position.pose.position.y = temp_path[i].y * map_.info.resolution + map_.info.origin.position.y;
        //     global_path.poses.push_back(position);
        // }
        // pub_path.publish(global_path);

        /* 清除路径path */
        path.clear();
        for (int i = 0 ; i <temp_path.size()-1; i++)
        {
            Pts_Generation(temp_path[i],temp_path[i+1]);
        }
        for(int i = 0;i < path.size();i++)
        {
            position.pose.position.x = path[i].x * map_.info.resolution + map_.info.origin.position.x;
            position.pose.position.y = path[i].y * map_.info.resolution + map_.info.origin.position.y;
            global_path.poses.push_back(position);
        }
        pub_path.publish(global_path);

        // int count=0;
        // int count_a = 0;
        // double last_x ,last_y;
        // last_x = 0, last_y = 0;
        // for(int i = 0;i<path.size();i++)
        // {
        //     path[i].path_x = path[i].x * map_.info.resolution + map_.info.origin.position.x;
        //     path[i].path_y = path[i].y * map_.info.resolution + map_.info.origin.position.y;
        // }
        // for(int i= path.size()-1 ; i>=0 ; i--)
        // {
        //     count++;
        //     position.pose.position.x = path[i].path_x;
        //     position.pose.position.y = path[i].path_y;

        //     if( count % 5 == 0)
        //     {
        //         global_path.poses.push_back(position);
        //         count_a++;
        //         double dis_det = sqrt ( (position.pose.position.x - last_x)*(position.pose.position.x - last_x) 
        //         + (position.pose.position.y - last_y)*(position.pose.position.y - last_y) );
        //         cout<<"dis betweent 2 pts:"<< dis_det<<endl;
        //         last_x = path[i].path_x;
        //         last_y = path[i].path_y;
        //     }            
        // }
        // pub_path.publish(global_path);
        // cout <<"Count::"<<count_a<<endl;
        cout<<"End_A*"<<endl;
    }

}


int Astar::Line_Obstacle_Check(Node A ,Node B)
{
    if(A.x == B.x)  //K=infinite
    {
        if(B.y>A.y)
        {
            for (int i = A.y+1; i < B.y ; i++ )
            {
                if( ( node[i*width + A.x].state == -1) || 
                ( node[i*width + A.x].state > 30 ) )//num 有障碍物或者未知区域
                {
                    return 1 ;
                }
            }
        }
        else
        {
            for (int i = B.y+1; i < A.y ; i++ )
            {
                if( ( node[i*width + A.x].state == -1) || 
                ( node[i*width + A.x].state > 30 ) )//num 有障碍物或者未知区域
                {
                    return 1 ;
                }
            }
        }
    }
    else    
    {
        double K =  (float)(A.y - B.y) / (float)(A.x - B.x);
        double b =  A.y - K * A.x;
        int check_y;
        int Start_num_x,End_num_x ,Start_num_y,End_num_y;
        if(B.x > A.x)    //对x
        {
            Start_num_x = A.x;
            End_num_x = B.x;
        }
        else if(B.x < A.x) 
        {
            Start_num_x = B.x;
            End_num_x = A.x;
        }
        for (int i = Start_num_x + 1; i < End_num_x ; i++ )
        {
            check_y = K * i  + b ;
      
            if( ( node[(check_y * width + i)].state == -1) || 
                ( node[(check_y * width + i)].state > 30 ) )//num 有障碍物或者未知区域
                {
                    return 1 ;
                }
             
        }  
        if(K != 0)
        {
            if(A.y < B.y) 
            {
                Start_num_y = A.y;
                End_num_y = B.y;
            }
            else if(A.y > B.y) 
            {
                Start_num_y = B.y;
                End_num_y = A.y;
            }
            for (int i = Start_num_y+1; i < End_num_y ; i++ ) //对Y
            {
                check_y =  (i - b) /K; //K ! = 0
                if( ( node[(i * width + check_y)].state == -1) || 
                    ( node[(i * width + check_y)].state > 30 ) )//num 有障碍物或者未知区域
                    {
                        return 1 ;
                    }
                    
            }
        }
    }
    
    return 0 ;
    
}

void Astar::Pts_Generation(Node A ,Node B)
{
    double ptr_dis = 0.15 ; //点之间间隔0.15米
    int t;
    if(A.x == B.x)  //K=infinite  Y方向插值
    {
        int num = abs(A.y - B.y)  * map_.info.resolution / ptr_dis;
        if(B.y>A.y)
        {
            for (int i =1; i <=num ; i++)
            {
                t = A.y + (B.y-A.y) /num  * i ; 
                path.push_back(node[ (t * width + A.x) ]);
            }
        }
        else
        {
            for (int i =1; i <=num ; i++ )
            {
                t = A.y -  (A.y-B.y) / num  * i ; 
                path.push_back(node[ (int)(t * width + A.x)]);
            }
        }
    }
    else    
    {
        double K =  (float)(A.y - B.y) / (float)(A.x - B.x);
        double b =  (A.y - K * A.x);
        int check_y;
        if( abs(K)<1 )  //对X方向插值
        {
            int num = abs(A.x - B.x)  * map_.info.resolution / ptr_dis;
            if(A.x > B.x)
            for (int i = 1; i <=num ; i++)
            {
                t = A.x -  (A.x-B.x) / num  * i ; 
                check_y = K * t  + b ;
                path.push_back(node[ check_y * width + t ]);     
            } 
            else
            for (int i = 1; i <=num ; i++)
            {
                t = A.x +  (B.x-A.x) / num  * i ;
                check_y = K * t  + b ;
                path.push_back(node[ check_y * width + t  ]);     
            }  
        }   
        else  //Y方向插值
        {
            int num = abs(A.y - B.y)  * map_.info.resolution / ptr_dis;
            if(A.y > B.y)
            for (int i = 1; i <=num ; i++)
            {
                t = A.y - (A.y-B.y) /num  * i ; 
                check_y = (t - b) / K ;
                path.push_back(node[ t * width + check_y ]);     
            } 
            else
            for (int i = 1; i <=num ; i++ )
            {
                t = A.y + (B.y-A.y) /num  * i ; 
                check_y = (t - b) / K ;
                path.push_back(node[ t * width + check_y ]);     
            }  
        }        

    }

}
void Astar::Path_Smooth()
{
    //First 删除共线点
    for(int i = 0 ; i< temp_path.size()-2; i++)
    {   ///三点共线
        if( (temp_path[i].x - temp_path[i+1].x) * (temp_path[i+1].y - temp_path[i+2].y) ==
            (temp_path[i].y - temp_path[i+1].y) * (temp_path[i+1].x - temp_path[i+2].x)  )
            {
                temp_path.erase(temp_path.begin() + i + 1 ,temp_path.begin() + i + 2);
                i--;
            }
    }
    cout<< "first size :" <<temp_path.size()<<endl;
    //处理拐点
    for(int i = 0 ; i < temp_path.size()-2;)
    {
        if(Line_Obstacle_Check(temp_path[i],temp_path[i+2]) == 0)
        {
            temp_path.erase( temp_path.begin() +i+1  ,temp_path.begin() + i + 2);
        }
        else
        {
            i++;
        }
    }

    cout << "temp of path size is : "<<temp_path.size()<<endl;

}

void Astar::Algorithm()
{
    cout << "begin to plan path!!!!!!!!" << endl;
    
    //start point
    car_pos.G = 0 ;
    car_pos.H = Manhattandistance(car_pos,goal);
    car_pos.F = car_pos.G + car_pos.H;
    cout<<"CAR_POS___FFFF:"<<car_pos.F<<endl;
    car_pos.open_visit = 1;

    openlist.clear();
    path.clear();
    global_path.poses.clear();
    openlist.push_back(&car_pos);
    global_path.header.frame_id = "map";
    //判断起点终点间是否存在障碍物，若不存在，则路径数组只具有一个终点节点（带检查是否需要起点节点）。否则开启寻路算法
    // if( obstacle_check_Start2Goal() == 0  )
    // {
    //     //无障碍物

    //     geometry_msgs::PoseStamped position;

    //     for(int i = 0;i<path.size();i++)
    //     {
    //         position.pose.position.x = path[i].x * map_.info.resolution + map_.info.origin.position.x;
    //         position.pose.position.y = path[i].y * map_.info.resolution + map_.info.origin.position.y;
    //         global_path.poses.push_back(position);
    //     }
    //     pub_path.publish(global_path);
    // }

    // // 开启寻路算法    
    // else
    {
        Astar_Search_Algorithm();
    }
    //////////////////////test//////////////////////////////////////////////////////////
    PC_TEST.header.frame_id = "map";
    PC_TEST.points.clear();
    geometry_msgs::Point32 test_point;
    // for(int i = 0;i < node.size();i++)  //n convert to point[r,c]
    // {

    //     if (node[i].visit ==1)
    //     {
    //         test_point.x = node[i].x * map_.info.resolution + map_.info.origin.position.x;
    //         test_point.y = node[i].y * map_.info.resolution + map_.info.origin.position.y;
    //         PC_TEST.points.push_back(test_point);
    //     }

    // }

    for(int i = 0;i < path.size();i++)  //n convert to point[r,c]
    {

            test_point.x = path[i].x * map_.info.resolution + map_.info.origin.position.x;
            test_point.y = path[i].y * map_.info.resolution + map_.info.origin.position.y;
            PC_TEST.points.push_back(test_point);
        

    }
    PC_pub.publish(PC_TEST);

    ////////////////////////test////////////////////////////////////////////////////////

    
}

int Astar::minInOpenList()
{
    double temp = (*openlist.begin())->F;
    int ans =  (*openlist.begin())->num;
    list<Node*>::iterator i;

    for(i = openlist.begin();i != openlist.end();i++)
    {
        if( (*i)->F < temp)
        {
            temp = (*i)->F;
            ans = (*i)->num;
        }

    }

    //closelist flag
    for(i = openlist.begin();i != openlist.end();)
    {
        if( (*i)->num == ans )
        {
            (*i)->visit = 1;
            (*i)->open_visit = 0;
             i = openlist.erase(i); //record iterator ptr
             break;
        }
        else i++;
    }
    return ans;
}

int Astar::CheckPath()
{
    return 0;
    int ans = 1;
    double min_distance = 1000;
    int min_n = path.size() - 1;
    for(int i = 0 ; i<path.size();i++)
    {
        double det_x = path[i].x - car_in_map_->x();
        double det_y = path[i].y - car_in_map_->y();
        double distance = sqrt(det_x * det_x + det_y * det_y);
        min_distance = min_distance > distance ? distance:min_distance;
        min_n = min_distance > distance ? i:min_n;
    }
    if(min_distance > 1)
    {
        return 0;
    }
    if(min_n < path.size() - 1)
    {
        path.erase(path.end() - min_n + 1,path.end());
    }

    for(int i = 0 ; i < path.size(); i++)
    {
        int x0 = (path[i].x - map_.info.origin.position.x) / map_.info.resolution;
        int y0 = (path[i].y - map_.info.origin.position.y) / map_.info.resolution;
        if(map_.data[y0 * map_.info.width + x0] > 30)
        {
            ans = 0;
        }
    }
    return ans;
}

void Astar::PthreadOne()
{
    if(pthread_create(&m_tid,NULL,ThreadOne,(void*)this) != 0)
    {
        ROS_INFO("Start one thread failed!");
        return;
    }
}

void* Astar::ThreadOne(void * arg)
{
    Astar *ptr =(Astar*) arg;
    ptr->ThreadRunOne();
    return NULL;
}

void Astar::ThreadRunOne()
{
    //Algorithm();///////////////////////////////////////////////////////////////////////test
    while(ros::ok())
    {
        if(load_map_flag_ == 1 && goal_flag == 1)
        {
            //Algorithm();
            goal_flag = 0;

        }
        if(load_map_flag_ == 1 && CheckPath()==0 && goal_flag == 0 )
        {
            //Algorithm();
        }
        ros::Duration(1).sleep();
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Astar_path");
    Astar astar;
    return 0;
}
