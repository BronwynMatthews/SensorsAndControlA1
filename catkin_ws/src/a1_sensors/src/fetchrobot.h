#ifndef FETCHROBOT_H
#define FETCHROBOT_H

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>

class FetchRobotController {
public:
    FetchRobotController(ros::NodeHandle nh);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void separateThread();
    

private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_state_sub_;
    ros::Publisher joint_command_pub_;
    ros::Publisher joint_velocity_pub_;
    std::vector<std::string> joint_names; // Names of the robot's joints
    
};

#endif // FETCHROBOT_H
