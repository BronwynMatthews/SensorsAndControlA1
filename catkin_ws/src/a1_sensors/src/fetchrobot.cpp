#include <iostream>
#include "fetchrobot.h"
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>

FetchRobotController::FetchRobotController(ros::NodeHandle nh) : nh_(nh) {
    // Subscribe to joint_states topic to receive current joint positions
    joint_state_sub_ = nh_.subscribe("/joint_states", 1, &FetchRobotController::jointStateCallback, this);

    // Create a publisher to send joint position commands
    joint_command_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_commands", 1);
    // Create a publisher to send joint velocity commands
    joint_velocity_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_velocity_commands", 1);
}

void FetchRobotController::separateThread() {
    ros::Rate loop_rate(100.0); // Set the ros rate limiter to 5Hz

    while (ros::ok()){
        // Example: Move the robot's arm to a predefined joint position
        sensor_msgs::JointState joint_command;
        joint_command.name.resize(2);
        joint_command.name = {"l_wheel_joint", "r_wheel_joint"}; // Names of the robot's joints
        //ROS_INFO("Size: %zu", joint_command.position.size());
        joint_command.position.resize(2);

        joint_command.position[0] = -1.7;

        joint_command.position[1] = 1.5;


        // Publish the joint position command
        joint_command_pub_.publish(joint_command);

        // Example: Set joint velocities
        sensor_msgs::JointState joint_velocity_command;
        joint_velocity_command.name.resize(2);
        joint_velocity_command.name = {"l_wheel_joint", "r_wheel_joint"}; // Names of the robot's joints
        joint_velocity_command.velocity.resize(2);

        joint_velocity_command.velocity[0] = 1.0; // Desired velocity for joint_1 (rad/s)
        joint_velocity_command.velocity[1] = -0.5; // Desired velocity for joint_2 (rad/s)

        // Publish the joint velocity command
        joint_velocity_pub_.publish(joint_velocity_command);

        loop_rate.sleep();
    }
    
}


void FetchRobotController::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // Process joint state data, e.g., print joint angles.
    for (size_t i = 0; i < msg->position.size(); ++i)
    {
        ROS_INFO("Joint %zu: %f", i, msg->position[i]);
    }
}
