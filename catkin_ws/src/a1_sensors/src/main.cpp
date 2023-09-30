#include "ros/ros.h"
#include "fetchrobot.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread> 

int main(int argc, char **argv) {
    // ros::init(argc, argv, "a1_sensors");
    // ros::NodeHandle nh;

    // // Create an instance of FetchRobotController
    // FetchRobotController controller(nh);

    // // Start a separate thread
    // std::thread t(&FetchRobotController::separateThread, &controller);

    // ros::spin();
    // ros::shutdown();

    // // Wait for the thread to finish
    // t.join();

    // return 0;
    ros::init(argc, argv, "robotmain");
    ros::NodeHandle nh;
    ROS_INFO("Break 0.5");
    // Initialize MoveIt
    moveit::planning_interface::MoveGroupInterface move_group("arm");

    // Set a target joint configuration (modify as needed)
    std::vector<double> target_joint_values = {-1.0, 1.0, -0.5, 0.5, -1.0, 1.0, -0.5};
    ROS_INFO("Break 1");
    // Set the target joint values
    move_group.setJointValueTarget(target_joint_values);
    ROS_INFO("Break 2");
    ROS_INFO("Break 21");
    // Plan and execute the motion
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    ROS_INFO("Break 2.5");
    moveit::core::MoveItErrorCode success = move_group.plan(my_plan);
    ROS_INFO("Break 3");
    sleep(5.0);
    move_group.move();

    // if (success)
    // {
    //     ROS_INFO("Planning and executing the motion...");
    //     move_group.move();
    //     ROS_INFO("Motion executed successfully!");
    // }
    // else
    // {
    //     ROS_ERROR("Failed to plan the motion.");
    // }

    ros::shutdown();
    return 0;
}