__Automatically Source ROS2__
1. bashrc is the shell's startup script Opening it with nano: `nano ~/.bashrc`
2. Add `source /opt/ros/jazzy/setup.bash` in a new line at the very end of the file and save it
3. In a new terminal run: `source ~/.bashrc`
4. After this all new terminal sessions will have ROS2 environment automatically sourced

__Building the Robot__
1. Navigate to the folder (dev_ws in this case)
2. `colcon build --symlink-install`
3. `source /install/setup.bash`
4. `ros2 launch {robot_name} rsp.launch.py` (my_bot in this case)

__Launching Rviz2__ `rviz2`

__Launching Joint State Publisher__ `ros2 run joint_state_publisher_gui joint_state_publisher_gui`

__RPLidar__
1. `ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB0 -p frame_id:=lidar_frame -p angle_compensate:=true -p scan_mode:=Standard`
2. In a new terminal, run `Rviz2` and choose the `/scan` topic
3. Running `ls /dev/serial/by-id` , we get `usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0` as the id
4. Suggested to run: `sudo chmod 666 /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`
This grants all permissions for use of the RPLidar
5. Hence, the below command can be used instead for when RP2040 is also connected via serial
`ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 -p frame_id:=lidar_frame -p angle_compensate:=true -p scan_mode:=Standard`
6. To find all service calls on RPLidar, type 'ros2 service call' and press tab. The following are the service calls
7. With the rplidar.launch.py launch file, 
`ros2 launch my_bot rplidar.launch.py`
may directly be used.
8. If it gets stuck, in another terminal sessions: `killall rplidar_composition`

The Serial Address of the RP2040 by ID is- 
usb-Raspberry_Pi_Pico_E6617C93E311432C-if00

__Camera__


__ROS2_control__

__SLAM__

__Nav2__

