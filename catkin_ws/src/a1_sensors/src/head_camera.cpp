#include "head_camera.h"
#include <opencv2/opencv.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <tf/transform_listener.h>

// Constructor for head camera class
HeadCamera::HeadCamera() {
    // Subscribe to RGB image topic
    sub_rgb = nh_.subscribe("/head_camera/rgb/image_raw", 1, &HeadCamera::rgbCallback, this);
    // Subscribe to depth camera topic
    sub_depth = nh_.subscribe("/head_camera/depth_registered/points", 1, &HeadCamera::depthCallback, this);

    // Initialize the flag to false
    objectDetected = false;
    // Set the dection counter to zero
    detectionCounter = 0;
    // Set maximum number of detection readings
    maxDetections = 10;

    // Initialize the TF listener
    tfListener = new tf::TransformListener();
}

// Callback function for RGB image
void HeadCamera::rgbCallback(const sensor_msgs::ImageConstPtr& msg_rgb) {
   try {
        // Convert ROS image to OpenCV Image
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg_rgb, sensor_msgs::image_encodings::BGR8);
        // Extract the Open CV Image
        cv::Mat rgb_image = cv_ptr->image;

        // Check if the image has been detected less than maxDetections times
        if (detectionCounter < maxDetections) {
            // Detect the blue image center
            findAndPublishBlueObjectCenter(rgb_image);
            // Detect the Red image center
            findAndPublishRedObjectCenter(rgb_image);
            // Increment the detection counter
            detectionCounter++;
        }
        // If we have reached maxDetections, you can stop further detections
        if (detectionCounter >= maxDetections) {
            // Set the flag to true
            objectDetected = true;
        }
    } 
    // Handle cv_bridge exceptions.
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception in rgbCallback: %s", e.what());
    }
}

// Callback function for depth camera
void HeadCamera::depthCallback(const sensor_msgs::PointCloud2ConstPtr& msg_depth) {
    pcl::PCLPointCloud2 pcl_pc2;
    // Convert ROS PointCloud2 message to PCL format.
    pcl_conversions::toPCL(*msg_depth, pcl_pc2);
    // Convert PCL point cloud to the 'cloud_' member variable.
    pcl::fromPCLPointCloud2(pcl_pc2, cloud_);
}

// Function to find and publish the center of a blue object in the image.
void HeadCamera::findAndPublishBlueObjectCenter(cv::Mat& image) {
    // Filter for blue color
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::Scalar lower_blue(100, 100, 100);
    cv::Scalar upper_blue(130, 255, 255);
    cv::Mat mask;
    cv::inRange(hsv, lower_blue, upper_blue, mask);

    // Find contours of the blue object
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    if (!contours.empty()) {
        std::vector<cv::Point> largestContour = contours[0];
        for (const auto& contour : contours) {
            if (cv::contourArea(contour) > cv::contourArea(largestContour)) {
                largestContour = contour;
            }
        }

        // Fit a bounding rectangle or polygon around the largest contour
        cv::RotatedRect boundingBox = cv::minAreaRect(largestContour);

        // Find the centroid of the bounding box
        cv::Point2f centroid = boundingBox.center;

        // Draw the bounding box and centroid on the image
        cv::Point2f vertices[4];
        boundingBox.points(vertices);
        for (int i = 0; i < 4; i++){
            cv::line(image, vertices[i], vertices[(i+1)%4], cv::Scalar(0, 255, 0));
        }
        cv::circle(image, centroid, 5, cv::Scalar(0, 0, 255), -1);

        // Use the centroid to find the corresponding 3D point
        if (centroid.x >= 0 && centroid.x < cloud_.width && centroid.y >= 0 && centroid.y < cloud_.height) {
            detectedCubePoint = cloud_.at(centroid.x, centroid.y);

            // Transform the point to the base_link frame
            transformPointToBaseLinkFrame(detectedCubePoint);

            // 'point' now contains the 3D coordinates of the detected center in the base_link frame
            ROS_INFO("Detected 3D center of Blue Cube (scan %d): x=%f, y=%f, z=%f", detectionCounter, detectedCubePoint.y, detectedCubePoint.z);
        }
    }
}

// Function to find and publish the center of a red object in the image.
void HeadCamera::findAndPublishRedObjectCenter(cv::Mat& image) {
    // Filter for red color
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::Scalar lower_red1(0, 100, 100);
    cv::Scalar upper_red1(10, 255, 255);
    cv::Scalar lower_red2(160, 100, 100);
    cv::Scalar upper_red2(180, 255, 255);
    cv::Mat mask1, mask2;
    cv::inRange(hsv, lower_red1, upper_red1, mask1);
    cv::inRange(hsv, lower_red2, upper_red2, mask2);
    cv::Mat mask = mask1 | mask2;

    // Find contours of the red object
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    if (!contours.empty()) {
        // Find the largest contour
        auto largestContour = std::max_element(contours.begin(), contours.end(),
                                                [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
                                                    return cv::contourArea(c1) < cv::contourArea(c2);
                                                });

        // Fit a bounding rectangle or polygon around the largest contour
        cv::RotatedRect boundingBox = cv::minAreaRect(*largestContour);

        // Find the centroid of the bounding box
        cv::Point2f centroid = boundingBox.center;

        // Draw the bounding box and centroid on the image
        cv::Point2f vertices[4];
        boundingBox.points(vertices);
        for (int i = 0; i < 4; i++){
            cv::line(image, vertices[i], vertices[(i+1)%4], cv::Scalar(0, 255, 0));
        }
        cv::circle(image, centroid, 5, cv::Scalar(0, 255, 0), -1);  // BGR color (0, 255, 0) represents green

        cv::imshow("Detected Objects", image);
        cv::waitKey(1);

        // Use the centroid to find the corresponding 3D point
        if (centroid.x >= 0 && centroid.x < cloud_.width && centroid.y >= 0 && centroid.y < cloud_.height) {
            detectedCylinderPoint = cloud_.at(centroid.x, centroid.y);

            // Transform the point to the base_link frame
            transformPointToBaseLinkFrame(detectedCylinderPoint);

            // 'point' now contains the 3D coordinates of the detected center in the base_link frame
            ROS_INFO("Detected 3D center of Red Cylinder (scan %d): x=%f, y=%f, z=%f", detectionCounter, detectedCylinderPoint.y, detectedCylinderPoint.z);
        }
    }
}

// Function to transform a 3D point to the base_link frame.
void HeadCamera::transformPointToBaseLinkFrame(pcl::PointXYZRGB& point) {
    tf::StampedTransform transform;
    try {
        // Lookup the transform from "head_camera_rgb_optical_frame" to "base_link" frame
        tfListener->lookupTransform("base_link", "head_camera_rgb_optical_frame", ros::Time(0), transform);

        // Create a tf::Vector3 with the point's coordinates
        tf::Vector3 point_tf(point.x, point.y, point.z);

        // Transform the point from the source frame to the target frame
        tf::Vector3 transformed_point = transform * point_tf;

        // Update the point's coordinates
        point.x = transformed_point.x();
        point.y = transformed_point.y();
        point.z = transformed_point.z();
    } catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
    }
}
// Getter method to get the detected 3D point of a cube
pcl::PointXYZRGB HeadCamera::getDetectedCubePoint() const {
    return detectedCubePoint;
}
// Getter method to get the detected 3D point of a cylinder
pcl::PointXYZRGB HeadCamera::getDetectedCylinderPoint() const {
    return detectedCylinderPoint;
}
