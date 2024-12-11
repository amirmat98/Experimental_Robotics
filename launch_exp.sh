konsole -e ros2 launch robot_urdf gazebo_aruco.launch.py &
sleep 5
konsole -e ros2 launch robot_urdf gazebo_aruco_controller.launch.py &
sleep 10
konsole -e ros2 run rqt_image_view rqt_image_view