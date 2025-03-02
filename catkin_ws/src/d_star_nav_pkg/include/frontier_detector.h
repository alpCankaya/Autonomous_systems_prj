#ifndef FRONTIER_DETECTOR_H
#define FRONTIER_DETECTOR_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <cmath>

class FrontierDetector {
public:
    FrontierDetector();
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

private:
    ros::Subscriber map_sub_;
    ros::Publisher frontier_pub_;
};

#endif
