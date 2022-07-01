# line_follow

## Overview

This ROS package implements a line following robot which detects the yellow line using the camera on Turtlebot3 Waffle Pi with the aid of image processing techniques and follows that line.

## Installation

Build from source, clone from this repository into your catkin workspace and compile the package using:
  
    git clone https://github.com/saiv20/line_follow.git
    cd ..
    catkin_make
    
Install the dependencies:

    sudo apt-get update
    sudo apt-get install ros-melodic-turtlebot3*
    
## Line Following Robot

1. Launch Gazebo with the world `line_follow1.world` and spawn Turtlebot3 Waffle Pi. We are using Turtlebot3 Waffle Pi instead of Turtlebot3 Burger as Waffle Pi comes with a built in camera, which is required in our case. To read more about Turtlebot3 click [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/). The inital position and pose of the robot is also set here.

	roslaunch line_follow follower1_world.launch
	
2. Launch the script that performs image processing and identifies the yellow line and follows it. The image processing is done using OpenCV and cv_bridge.

	roslaunch line_follow linefollow.launch
  

  
  
