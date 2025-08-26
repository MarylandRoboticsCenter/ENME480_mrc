This repository provides a Docker image for working with Universal Robots (UR) arms. The image is based on Ubuntu 22.04 and preconfigured for development using ROS 2 Humble.

## Overview

* The container includes two ROS2 workspaces: `ros_ur_driver` and `dev_ws`.
* `ros_ur_driver` workspace contains UR ROS2 drivers and UR Gazebo simulation packages. Don't use this workspace to add or edit ROS2 packages when running the container since all changes will be temporary. To make permanent changes, modify the Dockerfile and rebuild the image.
* `dev_ws` is the primary workspace for development when running the docker container. Two folders from the host machine are bind-mounted into the container:
  * `src` folder in the root of this repository is mounted into the container as `src` folder of `dev_ws` workspace. This allows to edit the code on the host system and compile it inside the container, while ensuring that all changes persist even after the container is stopped.
  * `config` folder in the root of this repository is mounted into the container as `config` folder of `dev_ws` workspace. This allows to easily edit the configuration parameters without rebuilding thge image or restarting the container.

## Workflow
* Build Docker image (run the command from the `docker` folder):
    ```
    userid=$(id -u) groupid=$(id -g) docker compose -f humble-enme480_ur3e-compose.yml build
    userid=$(id -u) groupid=$(id -g) docker compose -f humble-enme480_ur3e-nvidia-compose.yml build
    ```
* Start the container (run the command from the `docker` folder):
    ```
    docker compose -f humble-enme480_ur3e-compose.yml run --rm enme480_ur3e-docker
    docker compose -f humble-enme480_ur3e-nvidia-compose.yml run --rm enme480_ur3e-docker
    ```
* Once inside the container, use `tmux` to manage multiple panes:
  * `tmux`      # Start a new session
  * `Ctrl+A b`  # Split horizontally
  * `Ctrl+A v`  # Split vertically
* Use your preferred editor on the host machine to modify source code and config files. 
* Inside the container:
  * Compile your packages in `dev_ws` workspace using `colcon build`.
  * Start ROS2 nodes using `ros2 run ...` or `ros2 launch ...`
  
xacro enme480_ur3e_xacro.sdf > test.sdf


Used resources:
1. https://github.com/2b-t/docker-for-robotics
2. https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver - UR ROS2 official repository
3. https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/toc.html - UR ROS2 driver documentation