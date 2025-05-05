#!/bin/bash

set -e

# Imposta variabili per X11 e Qt
export DISPLAY=${DISPLAY:-":0"}
export QT_X11_NO_MITSHM=1
export QT_QPA_PLATFORM="xcb"

# Abilita accesso X11 per il root/docker (assicurati di eseguire `xhost +local:` sull'host prima di eseguire il container)
if command -v xhost &> /dev/null; then
    xhost +local:root
fi

# Esegui sourcing degli ambienti ROS e del workspace
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

# Avvia `roscore` in background
echo "Avvio di roscore in background..."
roscore > /dev/null 2>&1 &
sleep 2

# Rimuovi potenziali vecchi parametri `/run_id` da sessioni precedenti
rosparam delete /run_id || echo "Nessun run_id presente da eliminare."

# Avvia la simulazione con il package customizzato
echo "Avvio simulazione Universal Robot UR3 con gripper ed endoscopio in Gazebo..."
roslaunch my_ur3_setup ur_gripper.launch world:=empty.world &
GAZEBO_PID=$! # Salva il PID del processo Gazebo per gestione successiva

# Aspetta che Gazebo abbia il tempo di inizializzare completamente
sleep 5

# Mantieni il container attivo o lascia il controllo alla shell
wait
