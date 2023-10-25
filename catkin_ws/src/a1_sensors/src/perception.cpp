#include "perception.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

Perception::Perception() {
    sub_rgb = nh_.subscribe("/head_camera/rgb/image_raw", 1, &Perception::rgbCallback, this);
    sub_depth = nh_.subscribe("/head_camera/depth_registered/points", 1, &Perception::depthCallback, this);
}

void Perception::rgbCallback(const sensor_msgs::Image::ConstPtr& rgb_msg){
    // Convert the ROS image message to an OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Process the RGB image here
    cv::Mat rgb_image = cv_ptr->image;

    // Convert the RGB image to HSV color space
    cv::Mat hsv_image;
    cv::cvtColor(rgb_image, hsv_image, cv::COLOR_BGR2HSV);

    // Define HSV color range for red
    cv::Scalar lower_red(0, 100, 100);
    cv::Scalar upper_red(10, 255, 255);

    // Define HSV color range for blue
    cv::Scalar lower_blue(100, 100, 100);
    cv::Scalar upper_blue(130, 255, 255);

    // Create masks to isolate red and blue objects
    cv::Mat red_mask, blue_mask;
    cv::inRange(hsv_image, lower_red, upper_red, red_mask);
    cv::inRange(hsv_image, lower_blue, upper_blue, blue_mask);

     // Perform object detection and centroid finding in red_mask and blue_mask
    cv::Moments red_moments = cv::moments(red_mask);
    cv::Moments blue_moments = cv::moments(blue_mask);

    // Calculate the centroids
    double red_cx = red_moments.m10 / red_moments.m00;
    double red_cy = red_moments.m01 / red_moments.m00;

    double blue_cx = blue_moments.m10 / blue_moments.m00;
    double blue_cy = blue_moments.m01 / blue_moments.m00;

    // TF2 Transform Listener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Transform centroid points to the base_link frame
    geometry_msgs::PointStamped point_in;
    point_in.header.frame_id = "head_camera_rgb_optical_frame";
    point_in.point.x = red_cx;  // Use red or blue centroid
    point_in.point.y = red_cy;
    point_in.point.z = 0.0;  // Assuming the object is in the XY plane of the camera

    geometry_msgs::PointStamped point_out;
    try {
        // Perform the transformation
        point_out = tfBuffer.transform(point_in, "base_link");
         ROS_INFO("Transformed Point in base_link frame (X, Y, Z): (%f, %f, %f)",
                 point_out.point.x, point_out.point.y, point_out.point.z);
    } catch (tf2::TransformException& ex) {
        ROS_WARN("Transform failed: %s", ex.what());
        return;
    }
    // Your object detection and centroid finding code goes here
}

void Perception::depthCallback(const sensor_msgs::PointCloud2::ConstPtr& depth_msg){
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*depth_msg, cloud);
} 

