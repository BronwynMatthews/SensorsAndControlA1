#include "ros/ros.h"
#include "arm_controller.h"
#include "head_camera.h"
#include <thread>

int main(int argc, char **argv) {
    ros::init(argc, argv, "robotmain");
    ros::NodeHandle nh;

    // Create an instance of ArmController
    ArmController controller(nh);

    // Start a separate thread
    std::thread t(&ArmController::separateThread, &controller);

    ros::spin();
    ros::shutdown();

    // Wait for the thread to finish
    t.join();
    return 0;
}