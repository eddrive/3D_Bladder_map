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
- **Docker Compose:** If the project requires multiple containers, install the appropriate version.

### Building the Docker Image
Docker Commands and Explanations
1. Build the Docker Image
```shellscript
docker build -t ur_driver_noetic .
```

Purpose: Creates a Docker image from the Dockerfile in the current directory.
Explanation:
docker build: Initiates the Docker build process.
-t ur_driver_noetic: Tags the image with the name ur_driver_noetic.
.: Specifies the current directory as the build context, which contains the Dockerfile and related resources.




2. Run the Docker Container
```shellscript
xhost +local:root
```
```shellscript
docker run -it --rm --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ur_driver_noetic:latest
```


Purpose: Starts a container from the built image with settings that support interactive use and GUI applications.
Explanation:
docker run: Command to create and start a new container.
-it: Combines interactive mode and pseudo-TTY allocation for proper terminal interaction.
--rm: Automatically removes the container after it stops.
--net=host: Uses the host’s network stack, essential for direct communication with the UR3 robotic arm.
-e DISPLAY=$DISPLAY: Exports the host’s display environment variable, allowing GUI applications to display on the host.
-v /tmp/.X11-unix:/tmp/.X11-unix: Mounts the X11 socket needed for GUI display.
ur_driver_noetic:latest: Specifies the image to use with the latest tag.




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




4. Launching the ROS Driver for the UR3 Robotic Arm
```shellscript
roslaunch ur_robot_driver ur3_bringup.launch robot_ip:=141.64.75.55 kinematics_config:=${HOME}/ur3_calibration.yaml
```

Purpose: Launches the ROS driver to initialize the UR3 robot, establishing communication and applying calibration settings.
Explanation:
roslaunch: ROS command to start nodes as defined in a launch file.
ur_robot_driver: Specifies the ROS package containing the UR3 driver.
ur3_bringup.launch: The launch file that sets up the UR3 robot driver.
robot_ip:=141.64.75.55: Parameter to define the IP address of the UR3 robotic arm.
kinematics_config:=${HOME}/ur3_calibration.yaml: Sets the path to the kinematics calibration file necessary for accurate robot movements.

### Additional ROS Commands
1. Launch RViz for UR Driver

```shellscript
roslaunch ur_robot_driver example_rviz.launch
```
Purpose: Starts an instance of RViz configured for visualizing and interacting with the UR robot driver data.

Explanation:

roslaunch: Initiates the launch process for ROS nodes using the specified launch file.
ur_robot_driver: References the ROS package related to the UR robot.
example_rviz.launch: Launch file configured to open RViz with pre-configured settings tailored for the UR driver.
2. Test Movement Command

``` shellscript
rosrun ur_robot_driver test_move
```
Purpose: Executes a test command from the ur_robot_driver package to perform movement routines, verifying the setup and connectivity with the UR robot.

Explanation:

rosrun: ROS command to directly run an executable from a given package.
ur_robot_driver: Specifies the package where the test executable is located.
test_move: The executable responsible for performing a movement test on the robot.
