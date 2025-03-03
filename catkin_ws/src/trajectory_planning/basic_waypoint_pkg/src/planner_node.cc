/*
 * Simple example that shows a trajectory planner using
 *  mav_trajectory_generation.
 *
 *
 * Launch via
 *   roslaunch mav_trajectory_generation_example example.launch
 *
 * Wait for console to run through all gazebo/rviz messages and then
 * you should see the example below
 *  - After Enter, it receives the current uav position
 *  - After second enter, publishes trajectory information
 *  - After third enter, executes trajectory (sends it to the sampler)
 */

#include  "ros/ros.h"
#include <planner.h>

#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_planner");
    ros::NodeHandle n;
	
    BasicPlanner planner(n);  // instantiate basic planner
    ros::Duration(1.0).sleep();

    for (int i = 0; i < 10; i++) {
        ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
    }

    ROS_INFO("Starting planner. It will:");
    ROS_INFO("1. Follow waypoints from parameter space until signal is received");
    ROS_INFO("2. After signal, create direct trajectories to goals received on topic");

    // Run the planner - this will handle both modes and run forever
    planner.run();
    
    // This line should never be reached as run() contains an infinite loop
    return 0;
}
