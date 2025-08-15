# 3D Bladder Mapping Project

This project aims to create a **3D map of the bladder** during an endoscopic examination, thereby facilitating diagnoses and improving clinical analysis. The entire process is automated using **Universal Robotics UR3 robotic arms** to precisely perform the mapping.

## Table of Contents
- [Project Description](#project-description)
- [Docker Usage Guide](#docker-usage-guide)


----



## Project Description

This section outlines the context and primary objectives:
- **Main Objective:** Develop a three-dimensional representation of the bladder during endoscopic examinations.
- **Clinical Benefits:**  
  - Enhanced visualization of anatomical structures.
  - Greater support for medical diagnoses.
  - Opportunity for more detailed analysis of pathologies.
- **Approach:**  
  - Acquisition of endoscopic data.
  - 3D reconstruction using advanced algorithms.
  - Automated data acquisition with Universal Robotics UR3 to ensure precision and reproducibility.
  - Interactive visualization to support clinical decisions.
- **Sample Workflow:**  
  1. Initialization of the UR3 robotic system.
  2. Execution of a predefined scan routine to cover the entire bladder surface.
  3. Collection and transmission of imaging data to the 3D reconstruction module.



----



## Docker Usage Guide

This section provides a step-by-step guide to setting up the project environment using Docker.

### Prerequisites

Ensure you have installed:
- **Docker:** Version 20.10 or later.

### Building the Docker Image
Docker Commands and Explanations
1. Build the Docker Image
```shellscript
docker build -t ros2-ur3 .
```

Purpose: Creates a Docker image from the Dockerfile in the current directory.
Explanation:
docker build: Initiates the Docker build process.
-t ros2-ur3e-driver: Tags the image with the name ur_driver_noetic.
.: Specifies the current directory as the build context, which contains the Dockerfile and related resources.




2. Run the Docker Container
```shellscript
xhost +local:root
```
```shellscript
 docker run -it --rm --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --device=/dev/video0:/dev/video2 --device=/dev/video1:/dev/video3 --gpus all ros2-ur3:latest

```


Purpose: Starts a container from the built image with settings that support interactive use and GUI applications.
Explanation:
docker run: Command to create and start a new container.
-it: Runs the container in interactive mode and allocates a pseudo-TTY for terminal interaction.
--rm: Automatically removes the container once it stops.
--net=host: Uses the host's network stack, essential for direct communication with the UR3 robotic arm.
-e DISPLAY=$DISPLAY: Sets the DISPLAY environment variable in the container to enable GUI applications.
-v /tmp/.X11-unix:/tmp/.X11-unix: Mounts the host’s X11 socket into the container, allowing GUI applications to display.
--device=/dev/video2:/dev/video2: Makes the host’s video2 device available inside the container (e.g., a camera).
--device=/dev/video3:/dev/video3: Makes the host’s video3 device available inside the container (e.g., a second camera).
ros2-ur3:latest: Specifies the Docker image to use.




3. Access the Running Container's Shell
```shellscript
docker exec -it $container name$ bash
```

Purpose: Opens an interactive bash shell within the running Docker container.
Explanation:
docker exec: Runs a command inside a running container.
-it: Ensures the session is interactive, allowing direct terminal input.
$container name$: Identifies the container by name or container ID, you can just pres tab and is filled automatically.
bash: The command to execute in the container, which starts a bash shell.




4.  Driver Initialization via Entrypoint:
In this setup, the entrypoint script automatically performs the calibration and uses the output as a parameter to launch the driver. Additionally, it publishes the camera image streams as ROS2 topics for vision-based applications.
Next Steps:
Start the "External Control" program from the robot’s control panel.
Wait for the terminal output:
"Robot connected to reverse interface. Ready to receive control commands."
Once you see this message in the terminal, the setup is complete, and the robot is ready to be controlled.

## Using MoveIt!
MoveIt! support is integrated with this driver, enabling advanced robot control and motion planning within a Dockerized environment.
### A. Initial MoveIt! Setup (First-Time Configuration)
If you need to create or modify a MoveIt! configuration for your robot, use the MoveIt! Setup Assistant. This is typically a one-time process for a given robot configuration.
1.  **Ensure the Driver is Running:**
    Start your driver as previously described in this `README`.
2.  **Access the Docker Container:**
    Open a **new terminal** and gain shell access to your running Docker container. Replace `<container_name_or_ID>` with the actual name or ID of your Docker container.
    ```bash
    docker exec -it <container_name_or_ID> bash
    ```
3.  **Source ROS and Workspace Setup Files:**
    Once inside the Docker container, you must source the ROS environment and your workspace setup files. This makes all ROS 2 commands and your custom packages available in the current shell session.
    ```bash
    source /opt/ros/humble/setup.bash
    source /workspace/bladder_mapper/install/setup.bash
    ```
4.  **Launch MoveIt! Setup Assistant:**
    Execute the following command to start the graphical setup assistant:
    ```bash
    ros2 launch moveit_setup_assistant setup_assistant.launch.py
    ```
    This will launch the MoveIt! Setup Assistant GUI, allowing you to generate or update your robot's MoveIt! configuration files (e.g., SRDF, kinematics.yaml, etc.).
### B. Launching the Configured MoveIt! Environment
Once your MoveIt! configuration is set up, you can launch the configured MoveIt! environment for planning and execution.
1.  **Ensure the Driver is Running:**
    Start your driver as previously described in this `README`.
2.  **Access the Docker Container:**
    Open a **new terminal** and gain shell access to your running Docker container. Replace `<container_name_or_ID>` with the actual name or ID of your Docker container.
    ```bash
    docker exec -it <container_name_or_ID> bash
    ```
3.  **Source ROS and Workspace Setup Files:**
    Once inside the Docker container, you must source the ROS environment and your workspace setup files. This makes all ROS 2 commands and your custom packages available in the current shell session.
    ```bash
    source /opt/ros/humble/setup.bash
    source /workspace/bladder_mapper/install/setup.bash
    ```
4.  **Launch MoveIt! Nodes:**
    Now that your environment is correctly set up within the container, launch the MoveIt! configuration, including RViz2, using the following command:
    ```bash
    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 launch_rviz:=true
    ```
    *   `ur_type:=ur5e`: Specifies the UR robot model (e.g., `ur5e`, `ur10e`, `ur3e`) for which to load the MoveIt! configuration. Adjust this if you are using a different UR model.
    *   `launch_rviz:=true`: Launches RViz2, providing a visual interface to monitor the robot's state and interact with MoveIt!'s planning capabilities.
This sequence of commands will correctly set up your environment and launch the MoveIt! configuration, allowing you to visualize the robot, plan motions, and monitor trajectory execution.

## Endoscope Setup
The endoscope must be connected to `/dev/video2` on the host system. 
### Camera Calibration and Image Publishing
The **entrypoint script automatically provides the endoscope publishing node with the YAML calibration file**, ensuring that camera intrinsic parameters and distortion coefficients are properly loaded for accurate image processing.
By default, the system publishes the **uncorrected (raw) endoscope image** to the topic `endoscope/image_raw`. This raw image contains lens distortion and may not be suitable for precision applications.
### Viewing the Raw Endoscope Feed
To visualize the raw endoscope feed, use:
```shell
ros2 run rqt_image_view rqt_image_view
```
Then select the topic endoscope/image_raw from the dropdown menu.

Image Rectification (Distortion Correction)
If you need the corrected/rectified image (with lens distortion removed), you can launch the image rectification node:
```shell
ros2 run image_proc rectify_node --ros-args \
    --remap image:=endoscope/image_raw \
    --remap camera_info:=endoscope/camera_info \
    --remap image_rect:=endoscope/image_rect
```
This command:

Subscribes to the raw image (endoscope/image_raw) and camera calibration info (endoscope/camera_info)
Applies distortion correction using the calibration parameters
Publishes the corrected image to endoscope/image_rect

## Launching the 3D Mapping System
### 1. Ensure Prerequisites are Running
Make sure the following systems are operational before starting the mapping:
- UR3 robot driver (see [Docker Usage Guide](#docker-usage-guide))
- Endoscope camera publishing to `/endoscope/image_raw`
- Robot transformations available in TF tree
### 2. Access the Docker Container
Open a new terminal and gain shell access to your running Docker container:
```bash
docker exec -it <container_name_or_ID> bash
```
### 3. Source ROS and Workspace Setup Files
```bash
source /opt/ros/humble/setup.bash
source /workspace/bladder_mapper/install/setup.bash
```
###4. Launch the 3D Mapping System
```bash
ros2 launch bladder_rtabmapper rtabmap_external_odom.launch.py
```
