#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Perception {
public:
    Perception();
    void rgbCallback(const sensor_msgs::Image::ConstPtr& rgb_msg);
    void depthCallback(const sensor_msgs::PointCloud2::ConstPtr& depth_msg); 
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_rgb;
    ros::Subscriber sub_depth;
};