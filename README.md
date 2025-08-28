This repository contains a Docker image for working with UR3e robotic arms in the ENME480 course at the University of Maryland, College Park. The image is based on Ubuntu 22.04 and preconfigured for development using ROS2 Humble.

## Overview

* The container includes two ROS2 workspaces: ROS2 driver workspace (`ros_ur_driver`) and development workspace (`enme480_ws`).
* `ros_ur_driver` workspace contains UR ROS2 drivers and UR Gazebo simulation packages. Don't use this workspace to add or edit ROS2 packages when running the container since all changes will be temporary. To make permanent changes, modify the Dockerfile and rebuild the image.
* `enme480_ws` is the primary workspace for development when running the docker container. Two folders from the host machine are bind-mounted into the container:
  * `src` folder in the root of this repository is mounted into the container as `src` folder of `enme480_ws` workspace. This allows to edit the code on the host system and compile it inside the container, while ensuring that all changes persist even after the container is stopped.
  * `config` folder in the root of this repository is mounted into the container as `config` folder of `enme480_ws` workspace. This allows to easily edit the configuration parameters without rebuilding thge image or restarting the container.

## Workflow for controlling UR3e arms in the RAL lab

* Start the container (run the command from the `docker` folder):
    ```
    docker compose -f humble-enme480_ur3e-compose.yml run --rm enme480_ur3e-docker
    ```
* Once inside the container, use `tmux` to manage multiple panes. Create 4 panes to work with an UR3e arm:
  * `tmux`      # Start a new session
  * `Ctrl+A b`  # Split horizontally
  * `Ctrl+A v`  # Split vertically
* Launch the UR3e driver in one of the terminals:
    ```
    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.77.22 kinematics_params_file:=${HOME}/enme480_ws/config/ur3e_mrc.yaml
    ```
* On the teaching pendant start the program that allows ROS2 communication:
    `
    Programs-->URCaps-->External Control-->Control_by_MRC_ur3e_pc
    `
* Build your development workspace:
    ```
    cd enme480_ws
    colcon build --symlink-install
    ```
* Source your development workspace:
    ```
    source install/setup.bash
    ```
* Launch MRC UR3e package in a diiferent `tmux` pane:
    ```
    ros2 launch ur3e_mrc ur3e_enme480.launch
    ```
* Launch your node to move the arm in another `tmux` pane: `ros2 run {your node name}` or `ros2 launch {your launch file}`


## Workflow for launching Gazebo simulation on personal computers

* (NOT ON LAB PCs) Build Docker image (run the command from the `docker` folder). This needs to be done every time the Docker file is changed. On lab PCs this step should be skipped since the working image already exists.
    ```
    userid=$(id -u) groupid=$(id -g) docker compose -f humble-enme480_ur3e-nvidia-compose.yml build
    ```
* Start the container (run the command from the `docker` folder):
    ```
    docker compose -f humble-enme480_ur3e-compose.yml run --rm enme480_ur3e-docker
    ```
  On personal PCs with nvidia GPUs run the following command instead:
    ```
    docker compose -f humble-enme480_ur3e-nvidia-compose.yml run --rm enme480_ur3e-docker
    ```
* *work in progress

    ```
    xacro enme480_ur3e_xacro.sdf > enme480_ur3e.sdf
    ```

Used resources:
1. https://github.com/2b-t/docker-for-robotics
2. https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver - UR ROS2 official repository
3. https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/toc.html - UR ROS2 driver documentation