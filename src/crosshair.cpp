#include "crosshair.hpp"

// TODO: 
// 1. Subscribe to a ROS topic sensor_msgs/Image
// 2. Draw a crosshair on the image
// 3. Publish the crosshair image to a new ROS topic sensor_msgs/Image and the crosshair center position to a new ROS topic geometry_msgs/Point 
Crosshair::Crosshair(ros::NodeHandle& nh) : _nh(nh), _it(_nh) 
{

}

Crosshair::~Crosshair()
{
}
