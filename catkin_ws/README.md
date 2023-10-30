# Fetch Robot Grasping Project

## Description

This project is focused on implementing a robot grasping system using a Fetch robot equipped with sensors, cameras, and arm control. The robot is designed to pick up and manipulate objects in its environment.

## Individual Contributions

The contributions of the project was split up evenly. Here are the general areas that each memeber worked on.
- **Jack (34%)**: Worked on object detection and locating a center point for cube and cylinder, transforming detected point from head camera optimal frame to base_link frame, implimented logic for opening and closing gripper
- **Bronwyn (33%)**: Created the MoveIt package, initialised the robot poses in the MoveIt package, Intigrated the MoveIt package into the project to control the arm, Implimented the logic for the arm control including the pick and place and implimented safety considerations to ensure safe robot movements. 
- **Sophie (33%)**: Set up the GitHub repository, Created the ROS workspace, managed version control of the system, created the workspace and models, ensured proper organisation of project files and directories

## Setup Requirements

Before running the code, ensure you have the following:

- ROS (Robot Operating System) installed on your system.
- Required ROS packages installed.
- C++ libraries and dependencies.
- Fetch robot hardware or a compatible simulator.

## ROS Packages and C++ Libraries

This project relies on the following ROS packages and C++ libraries:

- [moveit](https://moveit.ros.org/): A library for motion planning and manipulation.
- [OpenCV](https://opencv.org/): An open-source computer vision library.
- [PCL (Point Cloud Library)](https://pointclouds.org/): Used for point cloud processing.
- [tf](http://wiki.ros.org/tf): A package for working with coordinate transforms.

Please make sure these packages and libraries are installed correctly.

## Compiling and Running the Code

1. Clone the project repository to your ROS workspace.
2. Build the project using `catkin_make`:

   ```bash
   cd your_ros_workspace
   catkin_make
