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


---

## Script

we implement two different scripts that are located in `ros2_aruco` directory.

1- **`marker_detector`**: carries out the operation where the whole robot navigates to locate the markers.<br/>
2- **`camera_marker_detector`**: implements the behaviour where only the camera moves to find the markers.

---

## Logic

---

## Install and run

First of all, you need to download the repository with the following command inside your workspace:

    git clone https://github.com/amirmat98/Experimental_Robotics.git

Then, you have to checkout the `Assignment1` branch:

    git checkout Assignment1

From the root directory of your ROS2 workspace run the command:

    colcon build

Now, you have to install `konsole` with the command:

    sudo apt-get install konsole

Inside the `/src/Experimental_Robotics` of your root directory and use:

    chmod u+x launch_exp.sh
    
Finally, to run the code, type the following command:

    bash launch_exp.sh


## Testing

This video shows the robot moving around in the Gazebo and interacting with its environment. Viewers may see the robot's actions and reactions to its surroundings in action in the video.

https://github.com/amirmat98/Experimental_Robotics/blob/Assignment1/readme/Gazebo_rob.mp4