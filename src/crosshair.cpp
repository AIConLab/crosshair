#include "crosshair.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ros/package.h>

Crosshair::Crosshair(ros::NodeHandle& nh) : _nh(nh), _it(nh) 
{      
    _loadCrosshairImage(); // Load the crosshair image

    // Retrieve the topic name from the parameter server
    std::string image_topic;
    _nh.getParam("/image_topic", image_topic);

    // Setup subscriber and publisher
    _image_sub = _it.subscribe(image_topic, 1, &Crosshair::_imageCallback, this);
    _image_pub = _it.advertise("/image_with_crosshair", 1);
}

Crosshair::~Crosshair() {
}

void Crosshair::_loadCrosshairImage() {
    // Get the relative path from the parameter server
    std::string relative_path;
    if (!_nh.getParam("crosshair_image_path", relative_path)) {
        ROS_ERROR("Failed to get crosshair image path from parameters");
        return;
    }

    // Construct the full path using the package path
    std::string package_path = ros::package::getPath("crosshair");
    std::string full_path = package_path + "/" + relative_path;

    _crosshair_image = cv::imread(full_path, cv::IMREAD_UNCHANGED);
    if (_crosshair_image.empty()) {
        ROS_ERROR("Failed to load crosshair image from %s", full_path.c_str());
    }
}

void Crosshair::_imageCallback(const sensor_msgs::ImageConstPtr& msg) 
{
    try
    {
        cv::Mat frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        cv::Point center(frame.cols / 2, frame.rows / 2);
        cv::Mat resized_crosshair;
        cv::resize(_crosshair_image, resized_crosshair, cv::Size(50, 50));  // Adjust size as necessary
        resized_crosshair.copyTo(frame(cv::Rect(center.x - 25, center.y - 25, resized_crosshair.cols, resized_crosshair.rows)));

        // Convert back to ROS image and publish
        sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        _image_pub.publish(out_msg);
    } 

    catch (const cv_bridge::Exception& e) 
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}
