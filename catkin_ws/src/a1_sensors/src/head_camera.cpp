#include "head_camera.h"

HeadCamera::HeadCamera() {
    sub_rgb = nh_.subscribe("/head_camera/rgb/image_raw", 1, &HeadCamera::rgbCallback, this);
    sub_depth = nh_.subscribe("/head_camera/depth_registered/points", 1, &HeadCamera::depthCallback, this);
}

void HeadCamera::rgbCallback(const sensor_msgs::ImageConstPtr& msg) {
    ROS_INFO("Received RGB image with width: %d, height: %d", msg->width, msg->height);
}

void HeadCamera::depthCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    ROS_INFO("Received depth data with width: %d, height: %d", msg->width, msg->height);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "head_camera_test");
    HeadCamera headCamera;
    ros::spin();
    return 0;
}

// #include "head_camera.h"
// #include <iostream>

// // constructor
// HeadCamera::HeadCamera() {
//     sub_rgb.subscribe(nh_, "/head_camera/rgb/image_raw", 1);
//     sub_depth.subscribe(nh_, "/head_camera/depth_registered/points", 1);
// }



// // callback for camera readings
// void HeadCamera::callback(const ImageConstPtr& msg_rgb, const PointCloud2ConstPtr& msg_depth) {

//     ObjectInfo detectedObject;
    
//     detectedObject.location = detectColour(msg_rgb, detectedObject.color, detectedObject.shape);
//     detectedObject.depthLocation = getDepth(msg_depth, detectedObject.location);
    
//     geometry_msgs::PointStamped target; 
//     target.point.x = detectedObject.depthLocation.x;
//     target.point.y = detectedObject.depthLocation.y;
//     target.point.z = detectedObject.depthLocation.z;
    
//     target.header.frame_id = "head_camera_rgb_optical_frame";
//     target.header.stamp = ros::Time::now();
//     pubPoint_.publish(target);
    
//     // store this information or pass it to another class
//     // for now, just printing it
//     std::cout << "Detected " << detectedObject.color << " " << detectedObject.shape 
//               << " and 3D location: (" << target.point.x << ", " << target.point.y << ", " << target.point.z << ")\n";
// }

// // cv::Point HeadCamera:detectColourPoint {


// // }

// // get depth, do we need to add in local to global co-ords? TBD
// pcl::PointXYZ HeadCamera::getDepth(const PointCloud2ConstPtr& msg_depth, const cv::Point& centroid) {
//     pcl::PointCloud<pcl::PointXYZ> depth;
//     pcl::fromROSMsg(*msg_depth, depth);
//     depth.points.resize(depth.width * depth.height);
//     return depth.at(centroid.x, centroid.y);
// }