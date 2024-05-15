#include "ros/ros.h"
#include "crosshair.hpp"

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "crosshair_node");
    ros::NodeHandle nh;

    Crosshair crosshair(nh);

    ros::spin();
    return 0;
}
