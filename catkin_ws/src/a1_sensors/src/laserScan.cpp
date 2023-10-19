// class to detect objects using a laser sccan
#include <iostream>
#include "laserScan.h"
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>    // fix this
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

// Constructor
laserScan::laserScan(sensor_msgs::LaserScan laserScan) : laserScan_(laserScan)
{

}

void LaserProcessing::newScan(sensor_msgs::LaserScan laserScan)
{
    laserScan_ = laserScan;
}

// Function to detect items 
bool laserScan::detectItem(sensor_msgs::LaserScan::ConstPtr laserScan)
{
    double laser_reading = laserScan->range_max;                                  // 
    int range_start = (laserScan->ranges.size() / 2) - (laserScanField / 2); // 
    int range_end = (laserScan->ranges.size() / 2) + (laserScanField / 2);   //
    for (unsigned int i = range_start; i <= range_end; i++)
    {
        if (laserScan->ranges.at(i) < laser_reading)
        {
            laser_reading = laserScan->ranges.at(i); // get laser reading and store it
        }
    }

    // if (segment size < allowable size) we assume it is either a cube or cylinder {
        // check colour
        // print colour of object into terminal
        // 
    // }

}


geometry_msgs::Point laserScan::detectObject(nav_msgs::Odometry odom, objectType type)
{
    countSegments(odom);

    // debugging
    if (segments.front().empty())
    {
        ROS_INFO("seg empty");
        return geometry_msgs::Point();
    }

    if (segments.empty())
    {
        ROS_INFO("temp segments is empty 2");
        return geometry_msgs::Point();
    }

    if (segments.back().empty())
    {
        ROS_INFO("temp segments is empty 3");
        return geometry_msgs::Point();
    }

    double shortest = 1000.0;
    double segmentLength = 0.0;
    int median = 0;

    shortestSegment.clear();

    geometry_msgs::Point midPoint;

    ROS_INFO_STREAM("size of vec" << segments.size());

    if ((abs(&segments.front().front().x - &segments.back().back().x) < 0.05) && (abs((&segments.front().front().y - &segments.back().back().y)) < 0.05))   {

        for (auto point : segments.front()) {
            segments.back().push_back(point);
               
        }

    segments.erase(segments.begin());

    }

    for (auto segment : segments)
    {
        if (segment.size() > 5)
        {
                
            for (auto point : segment)
            {
                
                // ROS_INFO_STREAM("point x: " << point.x << " y: " << point.y);
            }
            ROS_INFO("**********************************************************");
            segmentLength = hypot((segment.front().x - segment.back().x), (segment.front().y - segment.back().y));
            ROS_INFO_STREAM("Segment Length = " << segmentLength);

            if (segmentLength < shortest)
            {
                shortestSegment = segment;
                shortest = segmentLength;
            }
        }
    }

    median = shortestSegment.size() / 2;
    midPoint = shortestSegment.at(median);
    midPoint.z = odom.pose.pose.position.z;
    ROS_INFO_STREAM("PERSON midpoint x = " << midPoint.x << "  now   midpoint y = " << midPoint.y);
}




/////////// CONVERT FROM LOCAL TO GLOBAL UNSURE IF NEEDED

// geometry_msgs::Point laserScan::local2Global(double rangeBearing, double rangeDistance, nav_msgs::Odometry odom)
// {
//     geometry_msgs::Point p;
//     double yaw = tf::getYaw(odom.pose.pose.orientation);

//     // double yaw = tf::getYaw(aircraft.orientation);  // use orintation to get yaw angle

//     // calculate x and y from range and bearing
//     // account for oreintation and position of aircraft
//     p.x = odom.pose.pose.position.x + rangeDistance * cos(rangeBearing + yaw);
//     p.y = odom.pose.pose.position.y + rangeDistance * sin(rangeBearing + yaw);

//     // set z to 0
//     p.z = 0;

//     // return
//     return p;
// }



