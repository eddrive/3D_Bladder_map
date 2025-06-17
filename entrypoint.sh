#!/bin/bash

set -e

# Imposta le variabili per il server X11
export DISPLAY=${DISPLAY:-":0"}
export QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-"xcb"}
export QT_X11_NO_MITSHM=${QT_X11_NO_MITSHM:-"1"}

# Autorizza l'accesso al server X
if command -v xhost &> /dev/null
then
    xhost +local:root
fi

# Sorgente ROS2 Humble
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Errore: File setup.bash di ROS2 Humble non trovato!"
    exit 1
fi

# Sorgente del workspace ROS2
if [ -f /workspace/ros_ur_driver/install/setup.bash ]; then
    source /workspace/ros_ur_driver/install/setup.bash
else
    echo "Errore: Il workspace ROS2 non è stato costruito correttamente!"
    exit 1
fi

# Variabili per calibrazione e avvio driver
ROBOT_IP=${ROBOT_IP:-"141.64.75.55"}  # Default IP del robot
TARGET_FILENAME=${TARGET_FILENAME:-"${HOME}/my_robot_calibration.yaml"}
UR_TYPE=${UR_TYPE:-"ur3e"}  # Specifica il tipo di robot

# Passo 1: Avvia la calibrazione
echo "Avvio della calibrazione del robot con IP: ${ROBOT_IP}..."
ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=${ROBOT_IP} target_filename:=${TARGET_FILENAME}

echo "Calibrazione completata!"

# Passo 2: Avvia il driver del robot
echo "Avvio del driver con tipo robot: ${UR_TYPE} e file di calibrazione: ${TARGET_FILENAME}..."
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=${UR_TYPE} robot_ip:=${ROBOT_IP} kinematics_params_file:=${TARGET_FILENAME}

# Passa il controllo al comando specificato (ad esempio bash)
exec "$@"