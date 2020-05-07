#include "ros/ros.h"
#include "atr_msgs/table.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"table_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<atr_msgs::table>("atr"); 
    atr_msgs::table srv; 
    srv.request.distance = 10; 
    if(client.call(srv.request,srv.response))
    {
        ROS_INFO("Offset %f", (float)srv.response.offset); 
    }
    else
    {
        ROS_ERROR("Failed to call service table"); 
        return 1; 
    }
    return 0; 
}
