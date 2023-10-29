#include <iostream>
#include <thread>
#include <chrono>
#include "arm_controller.h"
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

// Constructur for ArmController class
ArmController::ArmController(ros::NodeHandle nh) : nh_(nh) {
    // Initialize MoveIt interfaces for Fetch
    move_arm_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>("arm");
    move_cam_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>("head");

    // Create gripper publisher
    gripper_publisher = nh_.advertise<control_msgs::GripperCommandActionGoal>("/gripper_controller/gripper_action/goal", 1);
    
    // Create publisher to send goals to head's joint trajectory controller
    head_goal_pub = nh_.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/head_controller/follow_joint_trajectory/goal", 1);

    // Create a publisher for sending velocity commands
    velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
}

void ArmController::separateThread() {
    // Set Distance to move forward
    double x_distance = 0;
    // Set How farm to turn
    double turn_distance = 0;
    // Set how farm to turn opposite direction
    double turn_negdistance = 0;
    // Set an extra short distance
    double shortdist = 0;
    // Set the offset of the gripper 
    double gripper_offset = 0.1;
    // Set the offset of the gripper link
    double endEffLink_offset = 0.17;
    // Set the depth error for the cube
    double depth_error_cube = 0.03;
    // Set the depth error for the cylinder
    double depth_error_cylinder = 0.02;

    // ********************** MOVE ROBOT BASE *********************** //
    if (turn_distance > 0) {
        turnBase(turn_distance, 1.0);
        turn_distance = 0.0;
    } else if (x_distance > 0) {
        moveBaseForward(x_distance, 3.0);
        x_distance = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    } else if (turn_negdistance > 0) {
        turnBase(turn_negdistance, -1.0);
        turn_negdistance = 0.0;
    } else if (shortdist > 0){
                moveBaseForward(shortdist, 0.5);
                shortdist = 0;
    } else {

        // Tilt the Head Camera down to view ojects
        move_cam_->setNamedTarget("tilt_down");
        move_cam_->move();

        // Initialise Head Camera Object
        HeadCamera headCamera;


        // MOVE ARM TO START POSE
        ROS_INFO("Moving the robot arm to the start pose...");
        move_arm_->setNamedTarget("start");
        moveArm();
        ROS_INFO("Robot arm is in the start pose.");

        // ******************** MOVE BLUE CUBE *********************** //

        // MOVE TO POS ABOVE BOX
        geometry_msgs::PoseStamped current_pose;
        current_pose = move_arm_->getCurrentPose("wrist_flex_link");
    
        // Get the detected point from the HeadCamera object
        pcl::PointXYZRGB detectedCubePoint = headCamera.getDetectedCubePoint();
        pcl::PointXYZRGB detectedCylinderPoint = headCamera.getDetectedCylinderPoint();

        ROS_INFO("Point of Blue Cube: x=%f, y=%f, z=%f", detectedCubePoint.x, detectedCubePoint.y, detectedCubePoint.z);
        ROS_INFO("Point of Red Cylinder: x=%f, y=%f, z=%f", detectedCylinderPoint.x, detectedCylinderPoint.y, detectedCylinderPoint.z);

        // Set the target pose
        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id = ("base_link");
        target_pose.pose.orientation = current_pose.pose.orientation;
        target_pose.pose.position.x = detectedCubePoint.x + depth_error_cube;
        target_pose.pose.position.y = detectedCubePoint.y;
        target_pose.pose.position.z = detectedCubePoint.z + endEffLink_offset + gripper_offset;
        // Set the target pose as the goal
        move_arm_->setPoseTarget(target_pose);
        // Move the arm
        moveArm();

        // MOVE DOWN ONTO BOX
        ROS_INFO("Moving the robot arm down to pick up the blue cube...");
        // Set the target pose
        target_pose.pose.position.z = target_pose.pose.position.z - gripper_offset;  // Set the desired position (z)
        // Set the target pose as the goal
        move_arm_->setPoseTarget(target_pose);
        moveArm();


        // CLOSE THE GRIPPER TO PICK UP BOX
        gripperControl("close");

        // MOVE ARM BACK UP
        target_pose.pose.position.z = target_pose.pose.position.z + gripper_offset;  // Set the desired position (z)
        // Set the target pose as the goal
        move_arm_->setPoseTarget(target_pose);
        moveArm();
        ROS_INFO("Blue Cube has successfully been grasped!");


        //MOVE ARM TO SEPERATE LOCATION
        target_pose.pose.position.x = target_pose.pose.position.x + 0.075;
        target_pose.pose.position.y = target_pose.pose.position.y - 0.3;
        target_pose.pose.position.z = target_pose.pose.position.z;  // Set the desired position (z)

        // Set the target pose as the goal
        move_arm_->setPoseTarget(target_pose);
        moveArm();

        ROS_INFO("Placing Blue Cube...");
        // MOVE ARM DOWN
        target_pose.pose.position.z = target_pose.pose.position.z - gripper_offset;  // Set the desired position (z)
        // Set the target pose as the goal
        move_arm_->setPoseTarget(target_pose);
        moveArm();

        //OPEN GRIPPER
        gripperControl("open");

        // MOVE ARM UP 
        target_pose.pose.position.z = target_pose.pose.position.z + gripper_offset;  // Set the desired position (z)
        // Set the target pose as the goal
        move_arm_->setPoseTarget(target_pose);
        moveArm();
        ROS_INFO("Blue Cube has Been Placed!");

        // ****************************** MOVE RED CYLINDER ********************** //

        target_pose.header.frame_id = ("base_link");
        target_pose.pose.position.x = detectedCylinderPoint.x + depth_error_cylinder;//0.6;  // Set the desired position (x)
        target_pose.pose.position.y = detectedCylinderPoint.y;//0.1;  // Set the desired position (y)
        target_pose.pose.position.z = detectedCylinderPoint.z + endEffLink_offset + gripper_offset;//0.724 + 0.17 + gripper_offset;  // Set the desired position (z)
        // Set the target pose as the goal
        move_arm_->setPoseTarget(target_pose);
        // Move the arm
        moveArm();

        // MOVE DOWN ONTO CYLINDER
        ROS_INFO("Moving the robot arm down to pick up the red cylinder...");
        // Set the target pose
        target_pose.pose.position.z = target_pose.pose.position.z - gripper_offset;  // Set the desired position (z)
        // Set the target pose as the goal
        move_arm_->setPoseTarget(target_pose);
        moveArm();


        // CLOSE THE GRIPPER TO PICK UP CYLINDER
        gripperControl("close");


        // MOVE ARM BACK UP
        target_pose.pose.position.z = target_pose.pose.position.z + gripper_offset;  // Set the desired position (z)
        // Set the target pose as the goal
        move_arm_->setPoseTarget(target_pose);
        moveArm();
        ROS_INFO("Red Cylinder has successfully been grasped!");


        //MOVE ARM TO SEPERATE LOCATION
        target_pose.pose.position.x = target_pose.pose.position.x + 0.0;
        target_pose.pose.position.y = target_pose.pose.position.y + 0.6;
        target_pose.pose.position.z = target_pose.pose.position.z;  // Set the desired position (z)

        // Set the target pose as the goal
        move_arm_->setPoseTarget(target_pose);
        moveArm();
        ROS_INFO("Placing Red Cylinder...");

        // MOVE ARM DOWN
        target_pose.pose.position.z = target_pose.pose.position.z - gripper_offset;  // Set the desired position (z)
        // Set the target pose as the goal
        move_arm_->setPoseTarget(target_pose);
        moveArm();

        //OPEN GRIPPER
        gripperControl("open");

        // MOVE ARM UP 
        target_pose.pose.position.z = target_pose.pose.position.z + gripper_offset;  // Set the desired position (z)
        // Set the target pose as the goal
        move_arm_->setPoseTarget(target_pose);
        moveArm();
        ROS_INFO("Red Cylinder has Been Placed!");
    }
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

    goal.goal.command.max_effort = 200.0; // Adjust the effort as needed

    // Publish the goal to the gripper action server
    gripper_publisher.publish(goal);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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