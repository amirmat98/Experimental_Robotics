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
This project incorporates ROSPlan-based planning for a mobile robot equipped with a camera and a laser scanner within a Gazebo environment featuring four distinct waypoints. Each waypoint contains one ArUco marker. The robot aims to traverse four distinct waypoints, identify the marker IDs at each waypoint, and subsequently go to the next waypoint, repeating this process until all markers are located, utilizing PDDL (Planning Domain Definition Language) activities. The robot should ultimately proceed to the place containing the marker with the lowest identifier. The technology employs a bespoke action interface to regulate the robot's behavior according to planning outcomes. While performing its mission, the robot must circumvent obstacles, including walls and ArUco marks, when navigating to a new place. Therefore, our system will employ gmapping as the Simultaneous Localization and Mapping (SLAM) technique to create a map and ascertain the robot's location within the environment. Additionally, the Move_base package will be utilized to facilitate autonomous navigation capabilities.

---

## The robot
The simulation use the ROSbot2R model, a mobile robot platform intended for integration with the Robot Operating System (ROS), as specified in the rosbot_description package. The robot is equipped with an RGBD camera to visualize the surrounding environment and identify objects of interest.

<br/>
<figure>
<img src="https://raw.githubusercontent.com/giangalv/EXPERIMENTAL_ROBOTICS_LABORATORY/refs/heads/main/b_FinalAssignment-main/readme_image/robot.jpg" style="width:60%">
</figure>
<br/>

---

## PDDL (Planning Domain Definition Language)

The Planning Domain Definition Language (PDDL) file located in this repository (</plansys2_assignment_pkg/pddl>) delineates the planning domain for the robotic system. This document delineates the kinds, predicates, and durative actions required for the robot to execute the designated tasks of marker localization and navigation.

### Features of the PDDL Domain

1. **Types**:
   - Defines the fundamental types of objects involved in the planning domain.
   - Examples:
     - `robot`: Represents the robot entity.
     - `waypoint`: Represents the navigation waypoints in the environment.

2. **Predicates**:
   - Logical conditions that define the state of the world.
   - Examples:
     - `(at-robby ?r - robot ?w - waypoint)`: Specifies that the robot `?r` is located at the waypoint `?w`.
     - `(explored ?w - waypoint)`: Indicates that a specific waypoint `?w` has been explored.
     - `(to_go ?w - waypoint)`: Marks a waypoint `?w` as a goal for navigation.

3. **Durative Actions**:
   - Actions that the robot can perform, including their preconditions and effects.
   - Examples:
     - `move`: Moves the robot from one waypoint to another.
     - `explore_waypoint`: Represents the exploration of a waypoint, detecting markers in the process.
     - `move_to_min`: Navigates the robot to the waypoint associated with the smallest ArUco marker ID.

### Example Workflow with PDDL

1. **Initialization**:
   - The robot starts at an unknown waypoint `(at-robby rob unknown)` and has the goal to explore all waypoints:  
     `(and (explored wp0) (explored wp1) (explored wp2) (explored wp3))`.

2. **Planning**:
   - Using the PDDL domain, the PlanSys2 planner generates a plan that satisfies the defined goal, leveraging the `move` and `explore_waypoint` actions.

3. **Execution**:
   - The plan is executed step-by-step, updating the predicates dynamically (e.g., marking waypoints as explored).
   - If the plan fails (e.g., a waypoint cannot be reached), a re-plan is triggered.

4. **Completion**:
   - Once all waypoints are explored, the mission concludes successfully.

### Integration with ROS2 and PlanSys2

The `mission_controller_node` initializes the PDDL problem and interfaces with the PlanSys2 planner to execute the generated plan. The PDDL file defines the foundational logic for the system, enabling the robot to make decisions dynamically based on the current state of the environment.


### Key Benefits

- **Modularity**: Actions and predicates can be extended or modified for additional tasks.
- **Re-planning Capability**: The domain supports dynamic re-planning in case of failures.
- **Robustness**: Logical predicates ensure accurate representation of the robot’s state and goals.


---

## Scripts

### 1. **`arucoDetector.cpp`**
- **Purpose**: Implements an ArUco marker detection system using OpenCV's ArUco module.
- **Key Features**:
  - Converts incoming ROS2 image messages to OpenCV images using `cv_bridge`.
  - Detects ArUco markers in the image, identifying their corners and IDs.
  - Publishes detected markers for further processing.
- **Core Methods**:
  - `ArucoDetector::detect`: Detects ArUco markers in a given image.
  - `ArucoDetector::getFrameAsImgMsg`: Converts processed OpenCV images back to ROS2 image messages.
- **Dependencies**: Requires `cv_bridge`, OpenCV, and ROS2 image processing libraries.


### 2. **`explore_waypoint_action_node.cpp`**
- **Purpose**: Implements an action node that explores waypoints using PlanSys2's action execution framework.
- **Key Features**:
  - Initializes an `ExploreWaypointAction` class to perform the "explore_waypoint" action.
  - Configures the action with parameters such as timeout and action name.
  - Runs the ROS2 event loop to handle the waypoint exploration lifecycle.
- **How It Works**:
  - Sets the `action_name` parameter to "explore_waypoint".
  - Monitors and manages the transition to the CONFIGURE state.
  - Uses PlanSys2's executor client for action management.
- **Dependencies**: Requires `plansys2_executor`, ROS2 libraries, and `arucoDetector` for marker detection.


### 3. **`mission_controller_node.cpp`**
- **Purpose**: Acts as the central controller for the mission, managing states, goals, and planning.
- **Key Features**:
  - Implements a state machine with the following states:
    - **`STARTING`**: Initializes the system and sets the exploration goal.
    - **`EXPLORE_WP`**: Executes the exploration plan, monitoring progress.
    - **`FINISHED_EXPLORING`**: Finalizes exploration and sets a secondary goal.
    - **`GO_TO_SMALLEST`**: Moves to the waypoint with the smallest ArUco ID.
    - **`FINISHED`**: Completes the mission.
  - Tracks visited waypoints and associates them with detected ArUco marker IDs.
  - Dynamically replans in case of execution failures.
- **Core Methods**:
  - `init`: Initializes the domain, problem, planner, and executor clients.
  - `show_progress`: Displays the mission's progress on the console.
  - `get_progress`: Updates visited waypoints and ArUco marker IDs.
  - `step`: Advances the state machine, executing and managing plans.
- **Dependencies**: Requires PlanSys2 libraries, ROS2 utilities, and `arucoDetector`.


### 4. **`move_action_node.cpp`**
- **Purpose**: Implements the "move" action for navigating the robot to specified waypoints.
- **Key Features**:
  - Initializes predefined waypoints (`wp0`, `wp1`, `wp2`, `wp3`).
  - Uses Nav2's action client to navigate the robot.
  - Subscribes to AMCL pose updates to track the robot's current position.
  - Provides feedback during navigation and monitors completion.
- **Core Methods**:
  - `current_pos_callback`: Updates the robot's current position based on AMCL data.
  - `on_activate`: Prepares the node for execution and sets navigation goals.
  - `getDistance`: Calculates the Euclidean distance between two positions.
- **Dependencies**: Requires `nav2_msgs`, ROS2 action libraries, and `geometry_msgs`.


### 5. **`move_to_min.cpp`**
- **Purpose**: Implements the "move_to_min" action, focusing on moving to the waypoint associated with the smallest detected ArUco ID.
- **Key Features**:
  - Shares much of its implementation with `move_action_node`.
  - Specifically targets the waypoint associated with the minimum-valued ArUco marker.
  - Monitors navigation progress and provides feedback.
- **Core Methods**:
  - `current_pos_callback`: Tracks the robot's current position.
  - `on_activate`: Configures the node for execution and sets navigation goals based on ArUco IDs.
  - `getDistance`: Computes the distance to the target waypoint.
- **Dependencies**: Similar to `move_action_node`, depends on `nav2_msgs`, ROS2 libraries, and `geometry_msgs`.


---

## Logic

This project demonstrates a robotic system that uses **ROS2**, **PlanSys2**, and **ArUco marker detection** to achieve the following tasks:
1. Explore a set of predefined waypoints in an environment.
2. Detect ArUco markers at waypoints and associate them with their locations.
3. Dynamically re-plan and adjust actions based on feedback from the system.
4. Navigate to the waypoint with the smallest detected ArUco marker ID.

The system uses a modular approach with clear state management, PDDL-based planning, and action execution. Below is a detailed breakdown of the logic.

### System Components

1. **Mission Controller (`mission_controller_node.cpp`)**:
   - Acts as the central orchestrator of the entire mission.
   - Manages high-level state transitions and interacts with other nodes to execute plans.
   - Uses a finite-state machine with the following states:
     - **`STARTING`**: Initialize the system and define the goal to explore all waypoints.
     - **`EXPLORE_WP`**: Execute the exploration plan by visiting waypoints.
     - **`FINISHED_EXPLORING`**: Once all waypoints are explored, prepare a secondary goal.
     - **`GO_TO_SMALLEST`**: Navigate to the waypoint with the smallest detected ArUco marker ID.
     - **`FINISHED`**: End the mission successfully.

2. **PDDL Domain**:
   - Specifies the planning domain with types, predicates, and durative actions.
   - Enables the system to dynamically compute plans based on the current state and goals.
   - Key actions include:
     - **`move`**: Navigate the robot to a specific waypoint.
     - **`explore_waypoint`**: Explore a waypoint and detect markers.
     - **`move_to_min`**: Navigate to the waypoint with the smallest ArUco ID.

3. **ArUco Marker Detection (`arucoDetector.cpp`)**:
   - Detects ArUco markers in the environment using an onboard camera.
   - Processes incoming image data, identifies markers, and publishes their IDs and locations.

4. **Action Nodes**:
   - **`move_action_node.cpp`**: Implements the "move" action for navigating to waypoints.
   - **`explore_waypoint_action_node.cpp`**: Implements the "explore_waypoint" action, including marker detection.
   - **`move_to_min.cpp`**: Implements the "move_to_min" action, which targets the waypoint associated with the smallest ArUco ID.


### Workflow Logic

#### 1. Initialization
- The **mission_controller_node** initializes the system:
  - Sets the initial position of the robot as `unknown`.
  - Defines waypoints (`wp0`, `wp1`, `wp2`, `wp3`) and their exploration goals using PDDL.
  - Starts the PlanSys2 planner and executor.

#### 2. Planning and Execution
- The system computes a plan to explore all waypoints:
  - Example goal: `(and (explored wp0) (explored wp1) (explored wp2) (explored wp3))`.
  - Actions like `move` and `explore_waypoint` are dynamically planned and executed.

#### 3. Marker Detection
- At each waypoint:
  - The robot detects ArUco markers using the `arucoDetector`.
  - Detected marker IDs are associated with the waypoint and stored for future reference.

#### 4. Re-Planning
- If a failure occurs (e.g., the robot cannot reach a waypoint):
  - The system dynamically updates the predicates (e.g., removing `at-robby` from the failed waypoint).
  - A new plan is computed to achieve the remaining goals.

#### 5. Final Goal: Navigate to Smallest ArUco Marker
- After all waypoints are explored, the system:
  - Determines the waypoint with the smallest detected ArUco marker ID.
  - Sets a new goal to navigate to this waypoint.
  - Executes the `move_to_min` action.

#### 6. Mission Completion
- Once the robot reaches the waypoint with the smallest ArUco ID, the mission concludes.
- The system transitions to the `FINISHED` state.


### Dynamic Re-Planning Example

1. **Failure Scenario**:
   - The robot attempts to move to `wp2` but fails.
   - The system removes the predicate `(to_go wp2)` and updates the problem definition.
   - A new plan is generated that skips `wp2` and focuses on the remaining waypoints.

2. **Marker Integration**:
   - When exploring a waypoint, the robot detects a marker with ID `5`.
   - This ID is associated with the waypoint and added to the system’s knowledge base.


### Key Features

1. **State Machine**:
   - Ensures organized control of the mission’s progress.
   - Handles transitions between initialization, exploration, re-planning, and completion.

2. **Modular Design**:
   - Each action (e.g., `move`, `explore_waypoint`, `move_to_min`) is implemented in a separate node, improving maintainability.

3. **Dynamic Planning**:
   - The system computes plans based on the current environment state and updates them as conditions change.

4. **Marker-Based Logic**:
   - ArUco markers provide a unique identifier for waypoints, allowing targeted navigation.


### Visualization

You can visualize the system’s behavior in:
- **Console Logs**: Monitor state transitions, plan execution, and feedback.
- **RViz**: (If configured) View the robot’s position, waypoints, and navigation progress.


---

## Install and run


The best way to run this package is to use this docker image. this container is a ubuntu 20 and [Ros2 foxy](https://docs.ros.org/en/foxy/index.html).

    https://hub.docker.com/r/carms84/noetic_ros2

Then You should install the following packages:

    apt install ros-foxy-ros2-control
    apt install ros-foxy-ros2-controllers
    apt install ros-foxy-nav2*
    apt install ros-foxy-plansys2-*
    apt install ros-foxy-slam-toolbox*

After that you should clone two packages in your workspace's src folder. You need to go to the src folder of your workspace and then use below commands to clone the packages.

    git clone -b Assignment2 https://github.com/amirmat98/Experimental_Robotics.git
    
    git clone https://github.com/CarmineD8/ros2_aruco.git

Now you should see two folder in your src folder with name of `Experimental_Robotics` and `ros2_aruco`.

then get back to the root of your workspace and build packages.

        cd ..

        source /opt/ros/foxy/setup.bash

        colcon build

        source install/setup.bash

Now you can run the package. Use each command in the seprate terminal to run the package.

+ `ros2 launch erl2_amirmat98 assignment2.launch.py`
+ `ros2 run erl2_amirmat98 mission_controller_node`

**Note:**
In the some cases, you may face  some problems in running the nodes. Please be patient and if the node does not start run again both the launchfile and the mission_manager node.

---

## Testing
A simulation demonstrating the robot's navigation and task execution is provided. This simulation highlights the robot's capabilities in planning and executing actions within its environment.



https://github.com/user-attachments/assets/c03ecc62-0d68-420e-8cf1-a9e5518f278e




---
