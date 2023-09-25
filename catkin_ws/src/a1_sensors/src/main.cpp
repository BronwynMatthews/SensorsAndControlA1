#include "ros/ros.h"
#include "fetchrobot.h"
#include <thread> 

int main(int argc, char **argv) {
    ros::init(argc, argv, "a1_sensors");
    ros::NodeHandle nh;

    // Create an instance of FetchRobotController
    FetchRobotController controller(nh);

    // Start a separate thread
    std::thread t(&FetchRobotController::separateThread, &controller);

    ros::spin();
    ros::shutdown();

    // Wait for the thread to finish
    t.join();

    return 0;
}