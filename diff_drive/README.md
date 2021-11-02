# GAZEBO DIFFERENTIAL DRIVE ROBOT
**Andru Liu**

## Overview 
This package contains a differential-drive robot that can be simulated in Gazebo and controlled by ROS. It also offers the ability to view the robot in RVIZ for configuration viewing. 

The robot moves in a straight line flipping over in order to change directions. 

## Usage Instructions 
To launch the package use the roslaunch command `roslaunch diff_drive <launch_file>`.

The launch files are listed below:
`ddrive_rviz.launch` - Launch file for visualizing the robot in RVIZ. Add the argument `` in order to launch the `joint_state_publisher_gui` to move the wheels. 

There is also an optional rviz config for viewing the robot with the odom frame as the fixed frame to see where the robot believes it to be based on its wheel odometry

`ddrive.launch`




## Configuration Instructions



## Videos