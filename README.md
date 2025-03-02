# Autonomous_systems_prj
# 1. Introduction
This is a project for the course LRG6300: Autonomous Systems at the Technical University of Munich.
This repository contains ROS packages for autonomous drone exploration using Unity in an unknown cave environment. The drone utilizes depth camera data to generate a point cloud, which is converted into a 3D Voxel Grid using the OctoMap library.
#Group Members
- Esra Kaplan Yılmaz: Navigation (Path Planning, Trajectory Generation, Manual Flight), Vision (Point Cloud Generation, Frontier Exploration, Octomapping),
- Alp Çankaya: Navigation (Path Planning, Trajectory Generation, Manual Flight), Vision (Point Cloud Generation, Frontier Exploration, Octomapping),  
- Erdem Ekinci: Navigation (Manual Flight), Vision (Point Cloud Generation, Lantern Detection & Logging)  
# Overview


# Installation
This setup is designed for **Ubuntu 20.04** with **ROS Noetic** installed.  

Follow [this guide to install ROS Noetic](http://wiki.ros.org/noetic/Installation), or run the following commands:  

```bash 
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
```bash 
sudo apt install curl
```
```bash 
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
```

Clone the repostitory: 
``` ```

## Requirements

    - depth_image_proc 
    - octomap
    - opencv2

`sudo apt install ros-<distro>-image ros-<distro>-octomap`

## How to run

`roslaunch main_launch main.launch` 
