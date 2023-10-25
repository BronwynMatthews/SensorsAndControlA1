#ifndef HEAD_CAMERA_H
#define HEAD_CAMERA_H

#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class HeadCamera {
public:
    HeadCamera();
    void rgbCallback(const sensor_msgs::ImageConstPtr& msg);
    void depthCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    std::string detectColor(const sensor_msgs::ImageConstPtr& msg_rgb);
    cv::Point3f get3DPoint(int u, int v);
    cv::Point2f findColorLocation(const cv::Mat& mask);
    void transformPointToBaseLink(const cv::Point3f &point_in_head_frame);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_rgb;
    ros::Subscriber sub_depth;
    sensor_msgs::PointCloud2ConstPtr latest_depth_msg_;
};

#endif  // HEAD_CAMERA_H

// #include <ros/ros.h>
// #include <geometry_msgs/PointStamped.h>
// #include <sensor_msgs/Image.h> 
// #include <sensor_msgs/PointCloud2.h>   
// #include <message_filters/subscriber.h> 
// #include <cv_bridge/cv_bridge.h>     
// #include <opencv2/opencv.hpp>         
// #include <pcl/point_cloud.h>          
// #include <pcl/point_types.h>          
// #include <pcl_conversions/pcl_conversions.h> 
// #include <string>
// #include <vector>

// class HeadCamera
// {
// public:
//     HeadCamera();
//     void callback(const sensor_msgs::ImageConstPtr& msg_rgb, const sensor_msgs::PointCloud2ConstPtr& msg_depth);
//     ObjectInfo scanForObject();  // This is the function that other classes will call


// private:
//     cv::Point detectColourAndShape(const sensor_msgs::ImageConstPtr& msg_rgb, std::string& color, std::string& shape);
//     pcl::PointXYZ getDepth(const sensor_msgs::PointCloud2ConstPtr& msg_depth, const cv::Point& centroid);

//     ros::NodeHandle nh_;
//     message_filters::Subscriber<sensor_msgs::Image> sub_rgb;
//     message_filters::Subscriber<sensor_msgs::PointCloud2> sub_depth;
//     ros::Publisher pubPoint_;  

//     struct HSVRange {
//         cv::Scalar min;
//         cv::Scalar max;
//     } objHSV_;

//     struct ObjectInfo {
//         cv::Point2d location;
//         pcl::PointXYZ depthLocation;
//         std::string color;
//         std::string shape;
//     };
// };