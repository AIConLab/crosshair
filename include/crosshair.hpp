#ifndef CROSSHAIR_HPP
#define CROSSHAIR_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class Crosshair 
{
public:
    Crosshair(ros::NodeHandle& nh);
    ~Crosshair();

private:
    ros::NodeHandle _nh;
    image_transport::ImageTransport _it;
    image_transport::Subscriber _image_sub;
    image_transport::Publisher _image_pub;

    cv::Mat _crosshair_image;

    void _imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void _loadCrosshairImage(void);
    void _overlay_with_alpha_channel(cv::Mat& frame, cv::Mat& overlay, cv::Point coordinate);
    void _overlay_no_alpha_channel(cv::Mat& frame, cv::Mat& overlay, cv::Point coordinate);
};

#endif // CROSSHAIR_HPP
