#!/bin/bash
set -e

# Imposta variabili per X11 e Qt
export DISPLAY=${DISPLAY:-":0"}
export QT_X11_NO_MITSHM=1
export QT_QPA_PLATFORM="xcb"

# Abilita accesso X11 per root/docker (ricorda di eseguire "xhost +local:" sull'host)
if command -v xhost &> /dev/null; then
    xhost +local:root
fi

# Esegui sourcing degli ambienti ROS e del workspace
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

# Avvia roscore in background e attendi che si stabilizzi
echo "Avvio di roscore in background..."
roscore > /dev/null 2>&1 &
sleep 5

# Rimuovi il parametro /run_id per eliminare residui da sessioni precedenti
echo "Eliminazione di /run_id..."
rosparam delete /run_id || echo "Nessun run_id presente da eliminare."
sleep 2

# Avvia la simulazione con il package customizzato
echo "Avvio simulazione Universal Robot UR3 con gripper ed endoscopio in Gazebo..."
roslaunch my_ur3_setup ur_gripper.launch world:=empty.world &
GAZEBO_PID=$!
sleep 10

# Mantieni il container attivo
wait
