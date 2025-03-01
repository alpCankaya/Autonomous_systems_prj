#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/Octomap.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Transform.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <queue>
#include <vector>
#include <limits>
#include <cmath>
#include "dstar_lite.h"
#include "frontier_detector.h"
#include "dstar_explorer.h"

DStarExplorer::DStarExplorer() {
    ros::NodeHandle nh;
    ros::Timer planning_timer;
    // Subscribers
    octomap_sub = nh.subscribe("/projected_map", 10, &DStarExplorer::mapCallback, this);
    frontier_sub = nh.subscribe("/frontiers", 10, &DStarExplorer::frontierCallback, this);
    current_state_sub = nh.subscribe("/current_state_est", 10, &DStarExplorer::currentStateCallback, this);
    lantern_detected_sub = nh.subscribe("/lantern_detected", 10, &DStarExplorer::lanternCallback, this);

    // Publishers
    path_pub = nh.advertise<nav_msgs::Path>("/dstar_path", 10);
    trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/desired_state", 10);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    planning_timer = nh.createTimer(ros::Duration(1.0), [this](const ros::TimerEvent&) { this->planPath(); });

    dstar = new DStarLite();
    map_received_ = false;
    frontier_received_ = false;
    current_pose_received_ = false;
    lantern_detected = false;
}

void DStarExplorer::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  map_ = *msg;  // Store the received map
  map_received_ = true;
  ROS_INFO("Map received and stored.");
}

void DStarExplorer::currentStateCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  current_pose_ = msg->pose.pose;
  current_pose_received_ = true;
}

void DStarExplorer::lanternCallback(const std_msgs::Bool::ConstPtr &msg) {
  lantern_detected = msg->data;
}


void DStarExplorer::frontierCallback(const geometry_msgs::PoseArray::ConstPtr &msg) {
    if (!current_pose_received_) return;

    double min_dist = std::numeric_limits<double>::infinity();
    int best_index = -1;
    
    double robot_x = current_pose_.position.x;
    double robot_y = current_pose_.position.y;
    
    tf::Quaternion q(
        current_pose_.orientation.x,
        current_pose_.orientation.y,
        current_pose_.orientation.z,
        current_pose_.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    double forward_x = cos(yaw);
    double forward_y = sin(yaw);

    for (size_t i = 0; i < msg->poses.size(); i++) {
        double fx = msg->poses[i].position.x;
        double fy = msg->poses[i].position.y;

        double dx = fx - robot_x;
        double dy = fy - robot_y;
        double dist = sqrt(dx * dx + dy * dy);

        double dot_product = (dx * forward_x) + (dy * forward_y);
        if (dot_product < 0) continue;

        int map_x, map_y;
        worldToMap(fx, fy, map_x, map_y);
        if (isNearObstacle(map_x, map_y, 3)) continue;

        if (dist < min_dist) {
            min_dist = dist;
            best_index = i;
        }
    }

    if (best_index != -1) {
        goal_pose_ = msg->poses[best_index];
        frontier_received_ = true;
        ROS_INFO("Selected forward frontier: (%.2f, %.2f)", goal_pose_.position.x, goal_pose_.position.y);
        planPath();
    } else {
        ROS_WARN("🚨 No valid forward frontiers found!");
    }
}

void DStarExplorer::planPath() {
  if (!map_received_ || !frontier_received_ || !current_pose_received_) return;

  static ros::Time last_plan_time = ros::Time(0);
  ros::Duration cooldown_time(1.0);
  if ((ros::Time::now() - last_plan_time) < cooldown_time) {
      ROS_WARN("⏳ Skipping path planning - cooldown active.");
      return;
  }
  last_plan_time = ros::Time::now();

  int start_x, start_y, goal_x, goal_y;
  worldToMap(current_pose_.position.x, current_pose_.position.y, start_x, start_y);
  worldToMap(goal_pose_.position.x, goal_pose_.position.y, goal_x, goal_y);
  goal_x_ = goal_x;
  goal_y_ = goal_y;

  ROS_INFO("📌 Planning path from (%d, %d) to (%d, %d)", start_x, start_y, goal_x, goal_y);

  nav_msgs::Path planned_path = computeDStarPath(start_x, start_y, goal_x, goal_y);

  if (planned_path.poses.empty()) {
      ROS_WARN("🚨 D* Lite failed! No valid path found.");
      fallbackMoveForward();
      return;
  }

  ROS_INFO("✅ D* Lite path generated with %ld waypoints", planned_path.poses.size());
  path_pub.publish(planned_path);

  // Convert and publish `/desired_state`
  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();
  trajectory_msg.header.frame_id = "map";

  // 🔥 **Take every 5th waypoint to reduce memory usage**
  for (size_t i = 0; i < planned_path.poses.size(); i += 5) {
      trajectory_msgs::MultiDOFJointTrajectoryPoint point;
      geometry_msgs::Transform transform;

      // **Ensure valid positions**
      if (std::isnan(planned_path.poses[i].pose.position.x) ||
          std::isnan(planned_path.poses[i].pose.position.y) ||
          std::isnan(planned_path.poses[i].pose.position.z)) {
          ROS_ERROR("🚨 Invalid waypoint detected (NaN) - Skipping index %ld!", i);
          continue;
      }

      transform.translation.x = planned_path.poses[i].pose.position.x;
      transform.translation.y = planned_path.poses[i].pose.position.y;
      transform.translation.z = planned_path.poses[i].pose.position.z;

      transform.rotation.w = 1.0;  // Valid quaternion

      point.transforms.push_back(transform);
      point.time_from_start = ros::Duration(i * 1.0);

      trajectory_msg.points.push_back(point);
  }

  if (trajectory_msg.points.empty()) {
      ROS_ERROR("🚨 Error: Trajectory message is empty! Aborting publish.");
      return;
  }

  trajectory_pub.publish(trajectory_msg);
  ROS_INFO("✅ Published trajectory with %lu waypoints to /desired_state", trajectory_msg.points.size());
}



void DStarExplorer::worldToMap(double wx, double wy, int &mx, int &my) {
  if (!map_received_) return;

  // Get actual map properties from OctoMap
  double origin_x = map_.info.origin.position.x; // Origin X
  double origin_y = map_.info.origin.position.y; // Origin Y
  double resolution = map_.info.resolution;      // Map resolution (from launch file: 1m)

  // Convert world coordinates to map grid indices
  mx = static_cast<int>(std::round((wx - origin_x) / resolution));
  my = static_cast<int>(std::round((wy - origin_y) / resolution));

  // Debug log
  ROS_INFO("worldToMap: World (%.2f, %.2f) -> Map (%d, %d)", wx, wy, mx, my);
}


void DStarExplorer::mapToWorld(int mx, int my, double &wx, double &wy) {
  if (!map_received_) return;

  // Get map properties
  double origin_x = map_.info.origin.position.x;
  double origin_y = map_.info.origin.position.y;
  double resolution = map_.info.resolution;

  // Convert map grid indices to world coordinates
  wx = (mx * resolution) + origin_x + (resolution / 2.0);
  wy = (my * resolution) + origin_y + (resolution / 2.0);

  // Debug log
  ROS_INFO("mapToWorld: Map (%d, %d) -> World (%.2f, %.2f)", mx, my, wx, wy);
}


bool DStarExplorer::isNearObstacle(int x, int y, int radius) {
    return false;  // Placeholder: Implement obstacle detection
}

nav_msgs::Path DStarExplorer::computeDStarPath(int start_x, int start_y, int goal_x, int goal_y) {
    nav_msgs::Path path;
    path.header.frame_id = "world";
    path.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = goal_x * 0.1;
    pose.pose.position.y = goal_y * 0.1;
    pose.pose.position.z = 0;

    path.poses.push_back(pose);
    return path;
}

void DStarExplorer::fallbackMoveForward() {
    double step_size = 0.5;
    tf::Quaternion q(
        current_pose_.orientation.x,
        current_pose_.orientation.y,
        current_pose_.orientation.z,
        current_pose_.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    goal_pose_.position.x = current_pose_.position.x + step_size * cos(yaw);
    goal_pose_.position.y = current_pose_.position.y + step_size * sin(yaw);
    goal_pose_.position.z = current_pose_.position.z;

    ROS_WARN("⚠️ No valid path found. Moving forward slightly instead.");
    planPath();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dstar_explorer");
    DStarExplorer explorer;
    ros::spin();
    return 0;
}
