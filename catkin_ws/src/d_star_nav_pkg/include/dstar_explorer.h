#ifndef DSTAR_EXPLORER_H
#define DSTAR_EXPLORER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>  // <-- Added for publishing goal
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Transform.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include "dstar_lite.h"

class DStarExplorer {
public:
    DStarExplorer();
    void frontierCallback(const geometry_msgs::PoseArray::ConstPtr &msg);
    void planPath();
    void fallbackMoveForward();
    void currentStateCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void lanternCallback(const std_msgs::Bool::ConstPtr &msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

private:
    // Subscribers
    ros::Subscriber octomap_sub, frontier_sub, current_state_sub, lantern_detected_sub;

    // Publishers
    ros::Publisher path_pub, trajectory_pub, cmd_vel_pub;
    ros::Publisher goal_pub_;  // <-- New publisher for /goal_position
    ros::Timer planning_timer;
    // Pointer to DStarLite implementation
    DStarLite* dstar;

    // State variables
    geometry_msgs::Pose current_pose_;
    geometry_msgs::Pose goal_pose_;
    bool map_received_;
    bool frontier_received_;
    bool current_pose_received_;
    bool lantern_detected;
    int goal_x_, goal_y_;

    // Occupancy Grid
    nav_msgs::OccupancyGrid map_;

    // Utility functions
    void worldToMap(double wx, double wy, int &mx, int &my);
    void mapToWorld(int mx, int my, double &wx, double &wy);
    bool isNearObstacle(int x, int y, int radius);
    nav_msgs::Path computeDStarPath(int start_x, int start_y, int goal_x, int goal_y);
};

#endif  // DSTAR_EXPLORER_H

