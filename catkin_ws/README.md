# Fetch Robot Grasping Project

## Description

This project is focused on implementing a robot grasping system using the Fetch robot equipped with RGB-D head camera and arm control for sorting different coloured and shaped objects. The robot is designed to pick up and manipulate objects in its environment to simulate a sorting task.

## Individual Contributions

The contributions of the project was split up evenly. Here are the general areas that each memeber worked on.
- **Jack (34%)**: Worked on object detection and locating a center point for cube and cylinder, transforming detected point from head camera optimal frame to base_link frame, implimented logic for opening and closing gripper
- **Bronwyn (33%)**: Created the MoveIt package, initialised the robot poses in the MoveIt package, Intigrated the MoveIt package into the project to control the arm, Implimented the logic for the arm control including the pick and place and implimented safety considerations to ensure safe robot movements. 
- **Sophie (33%)**: Set up the GitHub repository, Created the ROS workspace, managed version control of the system, created the workspace and models, ensured proper organisation of project files and directories

## Source Code
To view our source code, please locate to following directory 

```catkin_ws/src/a1_sensors/src```

### Code Structure
This codebase is part of a Pick and Place grasping project designed for sorting colored objects using a robot arm. The project involves various components, each contributing to the successful execution of the task.
#### Main (`main.cpp`)
- The main entry point for the program.
- Initializes the ROS (Robot Operating System) node and creates an instance of the `ArmController` class.
- Spawns a separate thread for robot control.
- Handles the high-level logic for controlling the robot.

#### Arm Controller (`arm_controller.cpp` and `arm_controller.h`)
- Manages the robot arm's movement and gripper control.
- Interfaces with MoveIt for motion planning and execution.
- Publishes commands to the arm and gripper controllers.
- Includes methods for controlling the robot base's movement (e.g., moving forward and turning).

#### Head Camera (`head_camera.cpp` and `head_camera.h`)
- Handles the robot's head camera, capturing RGB and depth images.
- Detects colored objects (blue cubes and red cylinders) in the images.
- Computes the 3D coordinates of the detected object centers.
- Provides transformation methods to convert 3D coordinates to the base_link frame.
- Publishes the detected object information.

# Setup Requirements
Before running the code, ensure you have the following:

## OS and Software
- Ubuntu 18.04
- C++
- ROS Melodic
- Gazebo
- fetch_gazebo_demo

## ROS Packages and C++ Libraries
- sensor_msgs
- std_msgs
- cv_bridge (OpenCV) 
- pcl_ros (PCL)
- tf
- moveit_core (MoveIt)
- moveit_planning_ros_interface (MoveIt)

Please make sure these packages and libraries are installed correctly.


## Compiling and Running the Code
Note that you will only be able to run the code if you have the fetch gazebo folder in you workspace

1. Clone the project repository to your ROS workspace.
```
git clone <https://github.com/BronwynMatthews/SensorsAndControlA1.git> /path/to/catkin_ws/src/
```
2. Build the project using `catkin_make`:

```
cd your_ros_workspace
catkin_make
```

3. Source the environment. Note this need to be done each time a terminal is open
```
source devel/setup.bash
```

4. Open the gazebo world file (Note this world and launch files are not included in this repository)
```
roslaunch fetch_gazebo fetch_sim.launch
```
5. Open a new terminal, locate to your catkin workspace and source the environment from step 3
6. Run the MoveIt group launch file
```
roslaunch fetch_moveit_config move_group.launch
```
7. Launch the program 
```
rosrun a1_sensors robotmain
```
