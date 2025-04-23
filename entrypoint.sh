#!/bin/bash
set -e

# Imposta le variabili per l'accesso al display X11
export DISPLAY=${DISPLAY:-":0"}
export QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-"xcb"}
export QT_X11_NO_MITSHM=${QT_X11_NO_MITSHM:-"1"}

# Autorizza l'accesso al server X per l'utente locale (in questo caso root)
xhost +local:root

# Sorgente dell'ambiente ROS e della workspace
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

# Avvio di roscore in background
echo "Avvio di roscore in background..."
roscore > /dev/null 2>&1 &
sleep 2  # Attende qualche secondo per assicurarsi che roscore sia attivo

# Pulisce eventuali run_id precedenti
rosparam delete /run_id || echo "Nessun run_id presente da eliminare"

# Avvia la calibrazione in background per 10 secondi (modifica il tempo se necessario)
echo "Avvio della calibrazione..."
roslaunch ur_calibration calibration_correction.launch robot_ip:=141.64.75.55 target_filename:=${HOME}/ur3_calibration.yaml &
CAL_PID=$!
sleep 10
echo "Terminazione della calibrazione..."
kill $CAL_PID || true  # Termina il processo di calibrazione

# Avvio in background del nodo che pubblica lo stream della camera
echo "Avvio nodo usb_cam per lo stream della camera..."
roslaunch usb_cam usb_cam-test.launch &
sleep 2

# Passa il controllo al comando specificato (es. bash)
exec "$@"

