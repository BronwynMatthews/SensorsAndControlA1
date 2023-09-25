#include <iostream>
#include "fetchrobot.h"

FetchRobotController::FetchRobotController(ros::NodeHandle nh) : nh_(nh) {

}

void FetchRobotController::separateThread() {
    std::cout << "Hello World" << std::endl;
}
