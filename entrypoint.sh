#!/bin/bash

set -e

# Set the variables for the X11 server
export DISPLAY=${DISPLAY:-":0"}
export QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-"xcb"}
export QT_X11_NO_MITSHM=${QT_X11_NO_MITSHM:-"1"}

# Allow access to the X server
if command -v xhost &> /dev/null; then
    xhost +local:root
fi

# Source ROS2 Humble environment
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "Sourcing ROS2 Humble environment..."
    source /opt/ros/humble/setup.bash
else
    echo "Error: ROS2 Humble setup.bash file not found!"
    exit 1
fi

# Source the workspace environment
if [ -f /workspace/ros_ur_driver/install/setup.bash ]; then
    echo "Sourcing custom workspace environment..."
    source /workspace/ros_ur_driver/install/setup.bash
else
    echo "Error: ROS2 workspace has not been built correctly!"
    exit 1
fi

# Check for required packages
echo "Checking for installed packages..."
required_packages=("ur_robot_driver" "ur_calibration" "ur3_endoscope_description")
for pkg in "${required_packages[@]}"; do
    if ! ros2 pkg list | grep -q $pkg; then
        echo "Error: Required package '$pkg' is missing from the workspace!"
        exit 1
    fi
done

# Variables for calibration and driver launch
ROBOT_IP=${ROBOT_IP:-"141.64.75.56"}  # Overrideable via environment variable
TARGET_FILENAME=${TARGET_FILENAME:-"/root/my_robot_calibration.yaml"}
UR_TYPE=${UR_TYPE:-"ur3"}
DESCRIPTION_PKG=${DESCRIPTION_PKG:-"ur3_endoscope_description"}
DESCRIPTION_FILE=${DESCRIPTION_FILE:-"ur3_endoscope.urdf.xacro"}

# Step 1: Start the calibration
echo "Starting robot calibration with ROBOT_IP=${ROBOT_IP}..."
ros2 launch ur_calibration calibration_correction.launch.py \
    robot_ip:=${ROBOT_IP} target_filename:=${TARGET_FILENAME}
echo "Calibration completed successfully!"

# Step 2: Start the robot driver
echo "Starting the Universal Robots driver with robot type ${UR_TYPE}..."
# ros2 launch ur_robot_driver ur_control.launch.py \
#     ur_type:=${UR_TYPE} \
#     robot_ip:=${ROBOT_IP} \
#     kinematics_params_file:=${TARGET_FILENAME} \

#Start the custom description launch file (uncomment if needed)
# ros2 launch ${DESCRIPTION_PKG} custom_ur_control.launch.py \
#     ur_type:=${UR_TYPE} \
#     robot_ip:=${ROBOT_IP} \
#     kinematics_params_file:=${TARGET_FILENAME} \
#     description_file:=${DESCRIPTION_FILE}

# Pass control to any additional commands specified at runtime
exec "$@"