#include "Commit_Goal.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Send_Destination");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Pose>("/own_destination", 1000);
    geometry_msgs::Pose goal;
    float x,y;
    while (ros::ok())
    {
       cout<<"please give the destination point : X,Y"<<endl;
       cin>>x>>y;
       goal.position.x = x;
       goal.position.y = y;
       pub.publish(goal);
    }
    
    return 0;
}


