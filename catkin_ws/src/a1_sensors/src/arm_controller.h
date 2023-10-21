#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <moveit/move_group_interface/move_group_interface.h>

class ArmController {
public:
    ArmController(ros::NodeHandle nh);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void BaseControl(const std::string& action, double distance, double velocity);
    void moveBase(const geometry_msgs::Twist& cmd_vel, double duration);
    void turnBase(double degrees, double velocity);
    void moveBaseForward(double distance, double velocity);
    void separateThread();
    void randFunction();

private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_state_sub_;
    ros::Publisher joint_command_pub_;
    ros::Publisher velocity_pub;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_arm_;
    //std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_base_;
};

#endif // ARM_CONTROLLER_H
