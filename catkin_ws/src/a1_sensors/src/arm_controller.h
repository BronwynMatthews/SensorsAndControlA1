#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include "head_camera.h"

class ArmController {
public:
    ArmController(ros::NodeHandle nh);
    void BaseControl(const std::string& action, double distance, double velocity);
    void moveBase(const geometry_msgs::Twist& cmd_vel, double duration);
    void turnBase(double degrees, double velocity);
    void moveBaseForward(double distance, double velocity);
    void gripperControl(const std::string& action);
    void separateThread();
    void moveArm();

private:

    ros::NodeHandle nh_;
    ros::Publisher velocity_pub;
    ros::Publisher gripper_publisher;
    ros::Publisher head_goal_pub;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_arm_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_cam_;

};

#endif // ARM_CONTROLLER_H
