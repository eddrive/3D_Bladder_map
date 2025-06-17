#!/bin/bash

set -e

# Set the variables for the X11 server
export DISPLAY=${DISPLAY:-":0"}  # Sets the DISPLAY environment variable to ":0" if not already set
export QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-"xcb"}  # Specifies the platform for the QPA environment
export QT_X11_NO_MITSHM=${QT_X11_NO_MITSHM:-"1"}  # Disables MIT-SHM usage for X11 compatibility

# Allow access to the X server
if command -v xhost &> /dev/null
then
    xhost +local:root  # Adds local root access to X server if xhost is available
fi

# Source ROS2 Humble environment
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash  # Sources the ROS2 Humble environment if setup.bash exists
else
    echo "Error: ROS2 Humble setup.bash file not found!"  # Displays an error message if setup.bash is missing
    exit 1  # Exits the script with an error code
fi

# Source the ROS2 workspace environment
if [ -f /workspace/ros_ur_driver/install/setup.bash ]; then
    source /workspace/ros_ur_driver/install/setup.bash  # Sources the workspace environment if setup.bash exists
else
    echo "Error: ROS2 workspace has not been built correctly!"  # Displays an error message if setup.bash is missing
    exit 1  # Exits the script with an error code
fi

# Variables for calibration and driver launch
ROBOT_IP=${ROBOT_IP:-"141.64.75.55"}  # Default IP for the robot
TARGET_FILENAME=${TARGET_FILENAME:-"${HOME}/my_robot_calibration.yaml"}  # Default calibration target file path
UR_TYPE=${UR_TYPE:-"ur3e"}  # Specifies the default type of robot

# Step 1: Start the calibration
echo "Starting robot calibration with IP: ${ROBOT_IP}..."  # Logs the start of calibration
ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=${ROBOT_IP} target_filename:=${TARGET_FILENAME}  # Executes the calibration launch file
echo "Calibration completed!"  # Logs the completion of calibration

# Step 2: Start the robot driver
echo "Starting the driver with robot type: ${UR_TYPE} and calibration file: ${TARGET_FILENAME}..."  # Logs the driver startup
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=${UR_TYPE} robot_ip:=${ROBOT_IP} kinematics_params_file:=${TARGET_FILENAME}  # Executes the driver launch file

# Pass control to the specified command (e.g., bash)
exec "$@"  # Executes the provided command (or bash by default)