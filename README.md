# Autonomous_systems_prj
1. Introduction
This repository contains ROS packages for autonomous drone exploration in an unknown cave environment using Unity. The drone utilizes depth camera data to generate a point cloud, which is converted into a 3D Voxel Grid using the OctoMap library.
# Installation

## Requirements

    - depth_image_proc 
    - octomap

`sudo apt install ros-<distro>-image ros-<distro>-octomap`

## How to run

`roslaunch main_launch main.launch` 
