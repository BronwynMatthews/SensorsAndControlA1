# Fetch Robot Grasping Project

## Description

This project is focused on implementing a robot grasping system using a Fetch robot equipped with sensors, cameras, and arm control. The robot is designed to pick up and manipulate objects in its environment.

## Individual Contributions

The contributions of the project was split up evenly. Here are the general areas that each memeber worked on.
- **Jack (34%)**: Worked on object detection and locating a center point for cube and cylinder, transforming detected point from head camera optimal frame to base_link frame, implimented logic for opening and closing gripper
- **Bronwyn (33%)**: Created the MoveIt package, initialised the robot poses in the MoveIt package, Intigrated the MoveIt package into the project to control the arm, Implimented the logic for the arm control including the pick and place and implimented safety considerations to ensure safe robot movements. 
- **Sophie (33%)**: Set up the GitHub repository, Created the ROS workspace, managed version control of the system, created the workspace and models, ensured proper organisation of project files and directories

# Source Code
To view our source code, please go to path 

src/a1_sensors/src

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

1. Clone the project repository to your ROS workspace.
```
git clone <> /path/to/catkin_ws/src/
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

4. Open the gazebo world file (Note this world file is not included in this repository)
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
