from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    assignment_path = FindPackageShare(package="exprob_dt").find("exprob_dt")
    urdf_path = os.path.join(assignment_path, "urdf")
    worlds_path = os.path.join(assignment_path, "worlds")
    rviz_config_path = os.path.join(assignment_path, "config")
    models_path = os.path.join(assignment_path, "models")

    gazebo_model_path = EnvironmentVariable(
        "GAZEBO_MODEL_PATH", default_value="")
    gazebo_model_path = [gazebo_model_path, ":", models_path]
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(
            ['xacro ', LaunchConfiguration('model')])}],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'robot',
                                   '-topic', '/robot_description'],
                        output='screen')

    broad = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"]
    )

    camera_velocity_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["camera_velocity_controller"]
    )

    run_assignment = Node(package="exprob_dt",
                          executable="assignment1",
                          # prefix=['gdbserver localhost:3000'],
                          prefix = ["lxterminal -e"],
                          output="screen")

    return LaunchDescription([
        SetEnvironmentVariable(name="GAZEBO_MODEL_PATH",
                               value=gazebo_model_path),
        DeclareLaunchArgument(name='model', default_value=os.path.join(urdf_path, "robot.xacro"),
                              description='Absolute path to robot urdf file'),
        # aruco_ros,
        robot_state_publisher_node,
        spawn_entity,
        camera_velocity_controller,
        broad,
        run_assignment,

        ExecuteProcess(
            cmd=['gazebo', '--verbose', worlds_path+'/aruco_world.world', '-s', "libgazebo_ros_factory.so", "-s", "libgazebo_ros_init.so"], output='screen'),
        # ExecuteProcess(
        #     cmd=['rviz2', '-d', rviz_config_path+'/rviz.rviz'],
        #     output='screen'),
    ])
