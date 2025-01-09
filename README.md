# Experimental_Robotics - Assignemnt 1
This repository contains the implementation of the **Assignment 1** for the Experimental Robotics Lab.

---

## Author 
AmirMahdi Matin - 5884715 <br/>
Sayna Arghideh - 5934809

---

## Table of Contents

1. [Description](#description)  
2. [The robot](#the-robot)  
3. [Script](#script)  
4. [Logic](#logic)  
5. [Code Overview](#code-overview)  
6. [Details](#details)
7. [Install and run](#install-and-run)


---

## Description

A ROS module that utilizes the camera of a robot to identify, track, and align markers. The robot is encircled by the markings. With this package, the robot can find markers, move to them, and highlight them on the camera stream. The code is written in two distinct ways. In the first 'camera_fix_controller.py' script, the entire vehicle chassis spins to pick up on the markers that have been set all around it. In the second piece of code, "camera_rotating_controller.py," the car's chassis remains stationary while the link carrying the camera spins.

<br/>
<figure>
<img src="https://raw.githubusercontent.com/amirmat98/Experimental_Robotics/refs/heads/Assignment1/readme/robot.png" style="width:60%">
</figure>
<br/>

---

## The robot

We have developed a flexible mobile platform that includes all the necessary components for perception and navigation. The *link_chassis* element defines the primary chassis, which is the structural backbone of the robot and where its vital components are housed. It also provides stability. Two wheels, *link_right_wheel* and *link_left_wheel*, are fastened to the chassis by means of continuous joints, which allow for seamless and uninterrupted movement.
A notable characteristic of this robot is its advanced camera system, consisting of the components link_camera_rot and camera_link. The camera is installed on a rotational joint, joint_camera_rot, enabling dynamic adjustments to its orientation. This functionality significantly enhances the robot's ability to perceive and interact with its surroundings.

---

## Script

we have implemented 3 classes to implement this project.
 **`arucoDetector`**: this class for detecting the presence of Markers.
 **`camMover`**: implements the behaviour where only the camera moves to find the markers.
 **`robotMover`**: implements the behaviour where the entire robot moves to find the markers.

---

## Logic

---

## Install and run

The best way to run this package is to use this docker image. this container is a ubuntu 20 and [Ros2 foxy](https://docs.ros.org/en/foxy/index.html).

    https://hub.docker.com/r/carms84/noetic_ros2

Then You should install the following packages:

    sudo apt install ros-foxy-ros2-control
    sudo apt install ros-foxy-ros2-controllers

After that you should clone two packages in your workspace's src folder. You need to go to the src folder of your workspace and then use below commands to clone the packages.

    git clone -b Assignment1 https://github.com/amirmat98/Experimental_Robotics.git
    
    git clone https://github.com/CarmineD8/ros2_aruco.git

Now you should see two folder in your src folder with name of `Experimental_Robotics` and `ros2_aruco`.

then get back to the root of your workspace and build packages.

        cd ..

        source /opt/ros/foxy/setup.bash

        colcon build

        source install/setup.bash

Now you can run the package.

    ros2 launch erl1_amirmat98 assignment1.launch.py





## Testing

This video shows the robot moving around in the Gazebo and interacting with its environment. Viewers may see the robot's actions and reactions to its surroundings in action in the video.


https://github.com/user-attachments/assets/1e6da857-6b55-4868-af94-0907fde7a94f

https://github.com/amirmat98/Experimental_Robotics/raw/refs/heads/Assignment1/readme/Assignment1_Simulation.mp4


