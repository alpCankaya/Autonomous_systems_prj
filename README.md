# Autonomous_systems_prj
# 1. Introduction
This is a project for the course LRG6300: Autonomous Systems at the Technical University of Munich.
This repository contains ROS packages for autonomous drone exploration using Unity in an unknown cave environment. The drone utilizes depth camera data to generate a point cloud, which is converted into a 3D Voxel Grid using the OctoMap library.
## Group Members
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
```bash 
sudo apt update
```
```bash 
sudo apt install ros-noetic-desktop-full
```
```bash 
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```

Clone the repostitory: 
```bash
git clone git@github.com:alpCankaya/Autonomous_systems_prj.git 
```
Open a terminal and navigate to 


## Dependencies 
The **depth_image_proc** package has been utilized for generating the Point Cloud. 
```bash
sudo apt install ros-noetic-depth-image-proc 
```
For the generation of the OctoMap, octomap and octomap_mapping packages have been utilized.
```bash
sudo apt-get install ros-noetic-octomap ros-noetic-octomap-mapping
```
    - depth_image_proc 
    - octomap
    - opencv2

`sudo apt install ros-<distro>-image ros-<distro>-octomap`

## How to run

Clone the repostitory: 
```bash
git clone git@github.com:alpCankaya/Autonomous_systems_prj.git 
```
Open a terminal and navigate to 

## Launching the Simulation 
To run the simulation: 
```bash
roslaunch main_launch main.launch
```

