#ifndef HEAD_CAMERA_H
#define HEAD_CAMERA_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

class HeadCamera {
public:
    HeadCamera();
    void rgbCallback(const sensor_msgs::ImageConstPtr& msg_rgb);
    void depthCallback(const sensor_msgs::PointCloud2ConstPtr& msg_depth);
    void findAndPublishBlueObjectCenter(cv::Mat& image);
    void findAndPublishRedObjectCenter(cv::Mat& image);
    void transformPointToBaseLinkFrame(pcl::PointXYZRGB& point);
    pcl::PointXYZRGB getDetectedCubePoint() const;
    pcl::PointXYZRGB getDetectedCylinderPoint() const;

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_rgb;
    ros::Subscriber sub_depth;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_;
    bool objectDetected;
    int detectionCounter;
    int maxDetections;
    tf::TransformListener* tfListener;
protected:
  pcl::PointXYZRGB detectedCubePoint;
  pcl::PointXYZRGB detectedCylinderPoint;
};

#endif  HEAD_CAMERA_H
