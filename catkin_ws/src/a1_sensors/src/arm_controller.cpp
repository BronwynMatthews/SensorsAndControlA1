#include <iostream>
#include <thread>
#include <chrono>
#include "arm_controller.h"
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>


ArmController::ArmController(ros::NodeHandle nh) : nh_(nh) {
    // Initialize MoveIt
    move_arm_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>("arm");
    move_gripper_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>("gripper");
    move_cam_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>("head");
    // Initialize MoveIt for the base group
    //move_base_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>("base");
    move_gripper_->setPlanningTime(10.0);
    //create gripper publisher
    gripper_publisher = nh_.advertise<control_msgs::GripperCommandActionGoal>("/gripper_controller/gripper_action/goal", 1);
    
    head_goal_pub = nh_.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/head_controller/follow_joint_trajectory/goal", 1);
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
    double x_distance = 0;//13.0;  // Desired X distance (adjust as needed)
    double turn_distance = 0;//2.0;  // Desired Y distance (adjust as needed)
    double turn_negdistance = 0;//1.5;
    double shortdist = 0;//0.5;
    double gripper_offset = 0.1;
    //while (ros::ok()) {
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
            move_cam_->setNamedTarget("tilt_down");
            move_cam_->move();


            // //MOVE ARM TO START POSE
            move_arm_->setNamedTarget("start");
            moveArm();

            // //MOVE TO POS ABOVE BOX
            geometry_msgs::PoseStamped current_pose;
            current_pose = move_arm_->getCurrentPose("wrist_flex_link");
            // // Set the target pose
            geometry_msgs::PoseStamped target_pose;
            //target_pose.header.frame_id = ("head_camera_rgb_optical_frame");
            target_pose.header.frame_id = ("base_link");
            target_pose.pose.orientation = current_pose.pose.orientation;
            target_pose.pose.position.x = 0.569478;//0.6;  // Set the desired position (x)
            target_pose.pose.position.y = 0.09940;//0.1;  // Set the desired position (y)
            target_pose.pose.position.z = 0.68997 + 0.17 + gripper_offset;//0.724 + 0.17 + gripper_offset;  // Set the desired position (z)
            // Set the target pose as the goal
            move_arm_->setPoseTarget(target_pose);
            // Move the arm
            moveArm();

            // MOVE DOWN ONTO BOX
            // Set the target pose
            // target_pose.pose.position.z = target_pose.pose.position.z - gripper_offset;  // Set the desired position (z)
            // // Set the target pose as the goal
            // move_arm_->setPoseTarget(target_pose);
            // moveArm();


            // CLOSE THE GRIPPER TO PICK UP BOX
            // gripperControl("close");


            // // MOVE ARM BACK UP
            // target_pose.position.z = target_pose.position.z + gripper_offset;  // Set the desired position (z)
            // // Set the target pose as the goal
            // move_arm_->setPoseTarget(target_pose);
            // moveArm();


            // //MOVE ARM TO SEPERATE LOCATION
            // target_pose.position.x = target_pose.position.x + 0.1;
            // target_pose.position.y = target_pose.position.y - 0.2;
            // target_pose.position.z = target_pose.position.z;  // Set the desired position (z)

            // // Set the target pose as the goal
            // move_arm_->setPoseTarget(target_pose);
            // moveArm();

            // // MOVE ARM DOWN
            // target_pose.position.z = target_pose.position.z - gripper_offset;  // Set the desired position (z)
            // // Set the target pose as the goal
            // move_arm_->setPoseTarget(target_pose);
            // moveArm();

            // //OPEN GRIPPER
            // gripperControl("open");

            // // MOVE ARM UP 
            // target_pose.position.z = target_pose.position.z + gripper_offset;  // Set the desired position (z)
            // // Set the target pose as the goal
            // move_arm_->setPoseTarget(target_pose);
            // moveArm();
        }
    //}
}

void ArmController::moveArm(){
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_arm_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        //ROS_INFO("Planning and executing the motion...");
        move_arm_->move();
        //ROS_INFO("Motion executed successfully!");
    } else {
        ROS_ERROR("Failed to plan the motion.");
    }
}

void ArmController::gripperControl(const std::string& action){
    control_msgs::GripperCommandActionGoal goal;
    if (action == "open"){
        // Define the gripper position
        goal.goal.command.position = 0.5;
    }
    else if (action == "close"){
        goal.goal.command.position = 0.0;
    }

    goal.goal.command.max_effort = 100.0; // Adjust the effort as needed

    // Publish the goal to the gripper action server
    gripper_publisher.publish(goal);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

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