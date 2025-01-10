# Experimental_Robotics - Assignment 2
This repository contains the implementation of **Assignment 2** for the Experimental Robotics Lab.

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
8. [Testing](#testing)

---

## Description
This assignment involves implementing a robot navigation and planning system using ROS2. The robot operates in a simulated environment, completing tasks by planning and executing actions through a mission controller. The system integrates the Planning System (PSys2) and leverages ROS2 services and actions to achieve its goals.

The mission includes dynamic goal handling and action execution, with the robot performing tasks such as navigation, object pickup, and placement. The implementation showcases an efficient approach to task planning and execution in a robotic framework.

<br/>
<figure>
<img src="https://raw.githubusercontent.com/amirmat98/Experimental_Robotics/refs/heads/Assignment2/readme/robot.png" style="width:60%">
</figure>
<br/>

---

## The robot
The robot design for this assignment retains a mobile platform with components essential for navigation and interaction with its environment. It includes:

- **Chassis:** The primary structural component (*link_chassis*), providing stability and housing vital components.
- **Wheels:** Two wheels (*link_right_wheel* and *link_left_wheel*), attached with continuous joints for seamless movement.
- **Camera:** An advanced camera system mounted on a rotational joint (*joint_camera_rot*), enabling dynamic perception adjustments to identify and interact with objects in the environment.

---

## Script
Three main classes are implemented to achieve the project goals:

1. **`MissionController`:** Manages the overall mission logic, handling goals and coordinating with other components.
2. **`ProblemExpertClient`:** Interfaces with the problem expert to add problem instances and goals.
3. **`ExecutorClient`:** Executes planned actions by communicating with action servers.

---

## Logic
The system follows this workflow:
1. Define problem instances and goals via the Problem Expert.
2. Generate plans using the Planner node.
3. Execute the plan step-by-step using the Executor node, coordinating actions like navigation, pickup, and placement.

---

## Install and run

### Prerequisites
Use the following Docker image for the environment setup:

    https://hub.docker.com/r/carms84/noetic_ros2

Install the necessary ROS2 packages:

    sudo apt install ros-foxy-ros2-control
    sudo apt install ros-foxy-ros2-controllers
    sudo apt install ros-foxy-planning-system

### Installation

1. Clone the repository and the required packages:

    ```bash
    git clone -b Assignment2 https://github.com/amirmat98/Experimental_Robotics.git
    git clone https://github.com/CarmineD8/ros2_aruco.git
    ```

2. Place the repositories in your ROS2 workspace's `src` folder:

    ```bash
    mv Experimental_Robotics ~/Second/A_WS/src
    mv ros2_aruco ~/Second/A_WS/src
    ```

3. Build the workspace:

    ```bash
    cd ~/Second/A_WS
    colcon build
    ```

4. Source the workspace:

    ```bash
    source /opt/ros/foxy/setup.bash
    source install/setup.bash
    ```

### Running the Package

To run the entire system, use:

    ```bash
    ros2 launch erl2_amirmat98 assignment2.launch.py
    ```

You can also run individual nodes for debugging purposes. Refer to the **Code Overview** section for details.

---

## Testing
A simulation demonstrating the robot's navigation and task execution is provided. This simulation highlights the robot's capabilities in planning and executing actions within its environment.


---

## Future Improvements
- Enhance dynamic obstacle avoidance during navigation.
- Extend action capabilities to include more complex behaviors.
- Integrate real-world perception techniques for task execution.

---

## License
This project is licensed under the MIT License. See the LICENSE file for details.
