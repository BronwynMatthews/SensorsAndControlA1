#include <iostream>
#include <thread>
#include <chrono>
#include "arm_controller.h"
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


ArmController::ArmController(ros::NodeHandle nh) : nh_(nh) {
    // Initialize MoveIt
    move_arm_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>("arm");
    // Initialize MoveIt for the base group
    //move_base_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>("base");
    // Set a lower planning time to make planning faster (adjust as needed)
    move_arm_->setPlanningTime(5.0);

    // Subscribe to joint_states topic to receive current joint positions
    joint_state_sub_ = nh_.subscribe("/joint_states", 1, &ArmController::jointStateCallback, this);

    // Create a publisher to send joint position commands
    joint_command_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_commands", 1);
    // Create a publisher for sending velocity commands
    velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
}
void ArmController::moveBase(const geometry_msgs::Twist& cmd_vel, double duration) {
    ros::Rate rate(5.0);
    ros::Time start_time = ros::Time::now();

    while (ros::Time::now() - start_time < ros::Duration(duration) && ros::ok()) {
        velocity_pub.publish(cmd_vel);
        rate.sleep();
    }
}

void ArmController::turnBase(double degrees, double velocity) {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = velocity;
    ROS_INFO("Turning...");
    moveBase(cmd_vel, degrees / std::abs(velocity));
}

void ArmController::moveBaseForward(double distance, double velocity) {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = velocity;
    cmd_vel.angular.z = 0.0;
    ROS_INFO("Moving straight. X Distance: %f", distance);
    moveBase(cmd_vel, distance / velocity);
}

void ArmController::separateThread() {
    double x_distance = 13.0;  // Desired X distance (adjust as needed)
    double turn_distance = 2.0;  // Desired Y distance (adjust as needed)
    double turn_negdistance = 1.5;
    double shortdist = 0.5;
    while (ros::ok()) {
        if (turn_distance > 0) {
            turnBase(turn_distance, 1.0); // Turn by 1 degree per cycle
            turn_distance = 0.0;
        } else if (x_distance > 0) {
            moveBaseForward(x_distance, 3.0); // Move forward with 3.0 m/s velocity
            x_distance = 0;
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        } else if (turn_negdistance > 0) {
            turnBase(turn_negdistance, -1.0); // Turn by -1 degree per cycle (negative direction)
            turn_negdistance = 0.0;
        }
        else if (shortdist > 0){
                    moveBaseForward(shortdist, 0.5);
                    shortdist = 0;
                }
         else {
            // Set the target pose
            geometry_msgs::PoseStamped target_pose;
            //target_pose.header.frame_id = "world";  // Set the reference frame
            target_pose.pose.position.x = 3.8;  // Set the desired position (x)
            target_pose.pose.position.y = 3.15;  // Set the desired position (y)
            target_pose.pose.position.z = 0.804140;  // Set the desired position (z)
            target_pose.pose.orientation.x = 0;  // Set the desired orientation (x)
            target_pose.pose.orientation.y = 0;  // Set the desired orientation (y)
            target_pose.pose.orientation.z = 0;  // Set the desired orientation (z)
            target_pose.pose.orientation.w = 0;  // Set the desired orientation (w)

            // Set the target pose as the goal
            move_arm_->setPoseTarget(target_pose);

            // Plan and execute the motion
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_arm_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if (success) {
                // Move to the target pose
                move_arm_->move();
                ROS_INFO("Motion executed successfully!");
            } else {
                ROS_ERROR("Failed to plan the motion.");
            }
        }
    }
}
void ArmController::randFunction(){
    double x_distance = 0;//13.0;  // Desired X distance (adjust as needed)
    double turn_distance = 0;//2.0;  // Desired Y distance (adjust as needed)
    double turn_negdistance = 0;//1.5;
    double shortdist = 0;//1;

    while (ros::ok()) {
        if (turn_distance > 0) {
            turnBase(turn_distance, 1.0); // Turn by 1 degree per cycle
            turn_distance = 0.0;
        } else if (x_distance > 0) {
            moveBaseForward(x_distance, 3.0); // Move forward with 3.0 m/s velocity
            x_distance = 0;
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        } else if (turn_negdistance > 0) {
            turnBase(turn_negdistance, -1.0); // Turn by -1 degree per cycle (negative direction)
            turn_negdistance = 0.0;
        }
         else {
            // Arm execution
            // Plan and execute the motion
            move_arm_->setNamedTarget("start");
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_arm_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if (success) {
                //ROS_INFO("Planning and executing the motion...");
                move_arm_->move();
                //ROS_INFO("Motion executed successfully!");
                if (shortdist > 0){
                    moveBaseForward(shortdist, 0.5);
                    shortdist = 0;
                }
            } else {
                ROS_ERROR("Failed to plan the motion.");
            }
        }
    }
}
void ArmController::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    // Process joint state data, e.g., print joint angles.
    for (size_t i = 0; i < msg->position.size(); ++i) {
        //ROS_INFO("Joint %zu: %f", i, msg->position[i]);
    }
}