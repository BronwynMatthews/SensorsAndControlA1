// Header for the laser scan class
#ifndef LASERSCAN_H
#define LASERSCAN_H

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <moveit/move_group_interface/move_group_interface.h>

class laserScan
{
public:
    


private:
    ros::NodeHandle nh_;

    
    sensor_msgs::LaserScan laserScan_;
    LaserProcessing(sensor_msgs::LaserScan laserScan);
    
    static const int laserScanField = 130;
    static const double laserScanLimit = 0.40;4geometry_msgs::Point detectObject(nav_msgs::Odometry odom, objectType type);
};



#endif //LASERSCAN.H