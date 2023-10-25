#include "head_camera.h"
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


HeadCamera::HeadCamera() {
    sub_rgb = nh_.subscribe("/head_camera/rgb/image_raw", 1, &HeadCamera::rgbCallback, this);
    sub_depth = nh_.subscribe("/head_camera/depth_registered/points", 1, &HeadCamera::depthCallback, this);
}

void HeadCamera::rgbCallback(const sensor_msgs::ImageConstPtr& msg) {
    std::string color = detectColor(msg);
    ROS_INFO("Detected color: %s", color.c_str());
    // Here you can add code to handle the detected color as needed
}

void HeadCamera::depthCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    latest_depth_msg_ = msg;
}

std::string HeadCamera::detectColor(const sensor_msgs::ImageConstPtr& msg_rgb) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg_rgb, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image;

        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

        // Define color ranges (e.g., Red and Blue)
        cv::Scalar lowerRed(0, 100, 100);
        cv::Scalar upperRed(10, 255, 255);
        cv::Scalar lowerBlue(100, 100, 100);
        cv::Scalar upperBlue(130, 255, 255);

        cv::Mat maskRed, maskBlue;
        cv::inRange(hsv, lowerRed, upperRed, maskRed);
        cv::inRange(hsv, lowerBlue, upperBlue, maskBlue);

        int redPixels = cv::countNonZero(maskRed);
        int bluePixels = cv::countNonZero(maskBlue);

        if (redPixels > bluePixels) {
            // Calculate the 3D location of the red color
            cv::Point2f redLocation = findColorLocation(maskRed);
            cv::Point3f red3DLocation = get3DPoint(redLocation.x, redLocation.y);
            ROS_INFO("3D location of red color: x=%f, y=%f, z=%f", red3DLocation.x, red3DLocation.y, red3DLocation.z);
            transformPointToBaseLink(red3DLocation);
            return "Red";
        } else {
            // Calculate the 3D location of the blue color
            cv::Point2f blueLocation = findColorLocation(maskBlue);
            cv::Point3f blue3DLocation = get3DPoint(blueLocation.x, blueLocation.y);
            ROS_INFO("3D location of blue color: x=%f, y=%f, z=%f", blue3DLocation.x, blue3DLocation.y, blue3DLocation.z);
            transformPointToBaseLink(blue3DLocation);
            return "Blue";
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception in detectColor: %s", e.what());
        return "Unknown";  // Error handling
    }
}

cv::Point2f HeadCamera::findColorLocation(const cv::Mat& mask) {
    cv::Moments m = cv::moments(mask, true);
    return cv::Point2f(m.m10/m.m00, m.m01/m.m00);
}

cv::Point3f HeadCamera::get3DPoint(int u, int v) {
    if (!latest_depth_msg_) {
        ROS_WARN("No depth data received yet");
        return cv::Point3f(NAN, NAN, NAN);
    }

    // Convert the (u, v) pixel coordinates to a linear index
    int index = v * latest_depth_msg_->row_step + u * latest_depth_msg_->point_step;

    float x, y, z;
    memcpy(&x, &latest_depth_msg_->data[index + latest_depth_msg_->fields[0].offset], sizeof(float));
    memcpy(&y, &latest_depth_msg_->data[index + latest_depth_msg_->fields[1].offset], sizeof(float));
    memcpy(&z, &latest_depth_msg_->data[index + latest_depth_msg_->fields[2].offset], sizeof(float));

    return cv::Point3f(x, y, z);
}

void HeadCamera::transformPointToBaseLink(const cv::Point3f &point_in_head_frame) {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    try {
        // Wait for the transform from "head_camera_rgb_optical_frame" to "base_link"
        geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("base_link", "head_camera_rgb_optical_frame", ros::Time(0));

        // Convert the 3D point to a geometry_msgs::PointStamped
        geometry_msgs::PointStamped point_in;
        point_in.header.stamp = ros::Time::now();
        point_in.header.frame_id = "head_camera_rgb_optical_frame";
        point_in.point.x = point_in_head_frame.x;  // Replace with the actual coordinates
        point_in.point.y = point_in_head_frame.y;
        point_in.point.z = point_in_head_frame.z;

        // Transform the point from head_camera_rgb_optical_frame to base_link
        geometry_msgs::PointStamped point_out;
        tf2::doTransform(point_in, point_out, transformStamped);

        // The transformed point is now in the "base_link" frame
        ROS_INFO("Transformed point in base_link frame: x=%f, y=%f, z=%f", point_out.point.x, point_out.point.y, point_out.point.z);
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Failed to transform point from head_camera_rgb_optical_frame to base_link: %s", ex.what());
    }
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