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
docker build -t ros2-ur3e-driver .
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
docker run -it --rm --net=host ros2-ur3e-driver:latest
```


Purpose: Starts a container from the built image with settings that support interactive use and GUI applications.
Explanation:
docker run: Command to create and start a new container.
-it: Combines interactive mode and pseudo-TTY allocation for proper terminal interaction.
--rm: Automatically removes the container after it stops.
--net=host: Uses the host’s network stack, essential for direct communication with the UR3 robotic arm.
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




4.  Driver Initialization via Entrypoint:
In this setup, the entrypoint script automatically performs the calibration and uses the output as a parameter to launch the driver.
Next Steps:
Start the "External Control" program from the robot’s control panel.
Wait for the terminal output:
"Robot connected to reverse interface. Ready to receive control commands."
Once you see this message in the terminal, the setup is complete, and the robot is ready to be controlled.
