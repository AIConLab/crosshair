#include "crosshair.hpp"
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <vector>

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

Crosshair::~Crosshair() 
{
}

void Crosshair::_loadCrosshairImage() 
{
    // Get the relative path from the parameter server
    std::string relative_path;
    if (!_nh.getParam("crosshair_image_path", relative_path)) 
    {
        ROS_ERROR("Failed to get crosshair image path from parameters");
        return;
    }

    // Construct the full path using the package path
    std::string package_path = ros::package::getPath("crosshair");
    std::string full_path = package_path + "/" + relative_path;

    _crosshair_image = cv::imread(full_path, cv::IMREAD_UNCHANGED);
    if (_crosshair_image.empty()) 
    {
        ROS_ERROR("Failed to load crosshair image from %s", full_path.c_str());
    }
}

void Crosshair::_overlay_with_alpha_channel(cv::Mat& frame, cv::Mat& overlay, cv::Point coordinate)
{
    ROS_INFO("Overlaying with alpha channel");

    std::vector<cv::Mat> channels;
    cv::split(overlay, channels);  // Split the overlay into channels
    cv::Mat rgb, alpha;

    // Create an array of the first three channels (RGB)
    std::vector<cv::Mat> rgbChannels;
    rgbChannels.push_back(channels[0]);
    rgbChannels.push_back(channels[1]);
    rgbChannels.push_back(channels[2]);
    cv::merge(rgbChannels, rgb);  // Merge the RGB channels back into an image
    alpha = channels[3];  // Use the fourth channel as the alpha channel

    cv::Rect roi(coordinate.x - rgb.cols / 2, coordinate.y - rgb.rows / 2, rgb.cols, rgb.rows);

    if (0 <= roi.x && 0 <= roi.width && roi.x + roi.width <= frame.cols && 0 <= roi.y && 0 <= roi.height && roi.y + roi.height <= frame.rows)
    {
        cv::Mat roiFrame = frame(roi);
        for (int y = 0; y < rgb.rows; y++)
        {
            for (int x = 0; x < rgb.cols; x++)
            {
                float alpha_value = alpha.at<unsigned char>(y, x) / 255.0;
                for (int c = 0; c < frame.channels(); c++)
                {
                    unsigned char foreground_pix = rgb.at<cv::Vec3b>(y, x)[c];
                    unsigned char background_pix = roiFrame.at<cv::Vec3b>(y, x)[c];
                    roiFrame.at<cv::Vec3b>(y, x)[c] = static_cast<unsigned char>(background_pix * (1.0 - alpha_value) + foreground_pix * alpha_value);
                }
            }
        }
        ROS_INFO("Applied alpha channel overlay");
    }
    else
    {
        ROS_WARN("ROI out of bounds");
    }
}

void Crosshair::_overlay_no_alpha_channel(cv::Mat& frame, cv::Mat& overlay, cv::Point coordinate)
{
    ROS_INFO("Overlaying without alpha channel");

    cv::Rect roi(coordinate.x - overlay.cols / 2, coordinate.y - overlay.rows / 2, overlay.cols, overlay.rows);

    if (0 <= roi.x && 0 <= roi.width && roi.x + roi.width <= frame.cols && 0 <= roi.y && 0 <= roi.height && roi.y + roi.height <= frame.rows)
    {
        overlay.copyTo(frame(roi));
        ROS_INFO("Applied overlay");
    }
    else
    {
        ROS_WARN("ROI out of bounds");
    }
}

void Crosshair::_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        ROS_INFO("Received image with size: %d x %d", frame.cols, frame.rows);

        // Proceed with further processing only if the image is valid
        if (frame.empty())
        {
            ROS_ERROR("Received an empty image");
            return;
        }

        // Calculate the center of the image
        cv::Point center(frame.cols / 2, frame.rows / 2);

        // Resize the crosshair image
        cv::Mat resized_crosshair;
        cv::resize(_crosshair_image, resized_crosshair, cv::Size(50, 50), 0, 0, cv::INTER_LINEAR);

        // If the crosshair image has an alpha channel, we need to blend it
        if (resized_crosshair.channels() == 4)
        {
            Crosshair::_overlay_with_alpha_channel(frame, resized_crosshair, center);
        }

        else
        {
            Crosshair::_overlay_no_alpha_channel(frame, resized_crosshair, center);
        }

        // Convert the frame back to a ROS image message and publish it
        sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
        _image_pub.publish(out_msg);
    }
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    catch (const cv::Exception& e)
    {
        ROS_ERROR("OpenCV exception: %s", e.what());
    }
}