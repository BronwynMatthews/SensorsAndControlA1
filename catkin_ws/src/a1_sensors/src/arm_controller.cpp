#include <iostream>
#include "arm_controller.h"
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <moveit/move_group_interface/move_group_interface.h>

ArmController::ArmController(ros::NodeHandle nh) : nh_(nh) {
    // Initialize MoveIt
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>("arm");

    // Set a lower planning time to make planning faster (adjust as needed)
    move_group_->setPlanningTime(5.0);

    // Subscribe to joint_states topic to receive current joint positions
    joint_state_sub_ = nh_.subscribe("/joint_states", 1, &ArmController::jointStateCallback, this);

    // Create a publisher to send joint position commands
    joint_command_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_commands", 1);
}

void ArmController::separateThread() {
    ros::Rate loop_rate(5.0); // Set the rate at which you want to control the arm (adjust as needed)

    while (ros::ok()) {
        // Example: Move the robot's arm to a predefined joint position
        std::vector<double> target_joint_values = {-1.0, 0.25, -0.25, 0.5, -0.9, 1.2, -0.5};
        move_group_->setJointValueTarget(target_joint_values);
        
        // Plan and execute the motion

        /////// NOT WORKING ////////
        // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        // bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        // if (success) {
        //     ROS_INFO("Planning and executing the motion...");
        //     move_group_->move();
        //     ROS_INFO("Motion executed successfully!");
        // } else {
        //     ROS_ERROR("Failed to plan the motion.");
        // }

        // Execute the motion
        ROS_INFO("Planning and executing the motion...");
        move_group_->move();
        ROS_INFO("Motion executed successfully!");
        loop_rate.sleep();
    }
}

void ArmController::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    // Process joint state data, e.g., print joint angles.
    for (size_t i = 0; i < msg->position.size(); ++i) {
        ROS_INFO("Joint %zu: %f", i, msg->position[i]);
    }
}