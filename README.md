# Autonomous_systems_prj
# 1. Introduction
This is a project for the course LRG6300: Autonomous Systems at the Technical University of Munich.
This repository contains ROS packages for autonomous drone exploration using Unity in an unknown cave environment. The drone utilizes depth camera data to generate a point cloud, which is converted into a 3D Voxel Grid using the OctoMap library.
## Group Members
- Esra Kaplan Yılmaz: Navigation (Path Planning, Trajectory Generation, Manual Flight), Vision (Point Cloud Generation, Frontier Exploration, Octomapping),
- Alp Çankaya: Navigation (Path Planning, Trajectory Generation, Manual Flight), Vision (Point Cloud Generation, Frontier Exploration, Octomapping),  
- Erdem Ekinci: Navigation (Manual Flight), Vision (Point Cloud Generation, Lantern Detection & Logging)  
- Can Uludogan: Vision (Point Cloud Generation, Octomapping), Navigation (Path Planning, Trajectory Generation, Frontier Exploration)  
- Reha Oguz Uslu: Vision (Point Cloud Generation, Octomapping), Navigation (Trajectory Generation, Frontier Exploration)
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
# Simulation Setup

1. **Copy Files:**  
   Copy the unpacked simulation files (see the **Sync&Share** folder for the files) into the folder `devel/lib/simulation`.

2. **Make It Executable:**  
   Don't forget to make the executable Simulation.x86_64 executable. (chmod +x catkin_ws/devel/lib/simulation/Simulation.x86_64)



## Dependencies 
The **depth_image_proc** package has been utilized for generating the Point Cloud. 
```bash
sudo apt install ros-<distro>-depth-image-proc 
```
For the generation of the OctoMap, octomap and octomap_mapping packages have been utilized.
```bash
sudo apt install ros-<distro>-image ros-<distro>-octomap
```
For the object detection from the semantic image view OpenCV and cv_bridge are needed.
```bash
sudo apt install ros-<distro>-vision-opencv
sudo apt install ros-<distro>-cv-bridge
```
For handling quaternion transformations used in coordinate conversions.
```bash
pip install numpy tf-transformations
```
## How to run
Clone the repostitory: 
```bash
git clone git@github.com:alpCankaya/Autonomous_systems_prj.git 
```
Open a terminal and navigate to 
```bash
cd catkin_ws 
```
Run the below commands to build the code: 
```bash
catkin init
```
```bash
catkin build
```
```bash
source devel/setup.bash
```
For the case if semantic_image_processor.py script is not executable, navigate to the catkin workspace and run: 
```bash
chmod +x src/semantic_image_processor/scripts/semantic_image_processor.py 
```
## Launching the Simulation 
To run the simulation in the terminal: 
```bash
roslaunch main_launch main.launch
```

