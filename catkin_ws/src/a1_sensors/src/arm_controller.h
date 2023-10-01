#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <moveit/move_group_interface/move_group_interface.h>

class ArmController {
public:
    ArmController(ros::NodeHandle nh);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void separateThread();

private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_state_sub_;
    ros::Publisher joint_command_pub_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

#endif // ARM_CONTROLLER_H
