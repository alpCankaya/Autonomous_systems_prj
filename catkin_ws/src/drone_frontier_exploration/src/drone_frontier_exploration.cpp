/**
 * @file drone_frontier_exploration.cpp
 * @brief Implementation of frontier-based exploration for autonomous drone navigation
 *
 * This node implements a frontier-based exploration algorithm for autonomous drone navigation.
 * It uses an octomap representation of the environment to identify frontier points (boundaries
 * between known free space and unknown space), clusters these points, and selects the centroid
 * of the largest cluster as the next exploration goal.
 */

#include <ros/ros.h>
#include "drone_frontier_exploration/Optics.hpp"
#include <algorithm>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>


class DroneFrontierExploration {
public:
  /**
   * @brief Constructor for the DroneFrontierExploration class
   *
   * Initializes ROS subscribers, publishers, and parameters for frontier exploration
   */
  DroneFrontierExploration();

private:
  // ROS communication
  ros::NodeHandle node_handle_;
  ros::Subscriber position_subscriber_;
  ros::Subscriber octomap_subscriber_;
  ros::Publisher goal_publisher_;
  ros::Timer exploration_timer_;

  // Octomap and exploration data
  octomap::OcTree::leaf_bbx_iterator octomap_iterator_;
  std::shared_ptr<octomap::OcTree> octomap_tree_{};
  octomap::point3d drone_position_;
  std::vector<geometry_msgs::Point> frontier_points_;
  int octomap_resolution_, max_exploration_distance_;
  std::vector<pcl::PointIndicesPtr> frontier_clusters_;

  /**
   * @brief Callback for drone position updates
   * @param msg Odometry message containing the drone's current position
   */
  void updateDronePosition(const nav_msgs::Odometry &msg);
  
  /**
   * @brief Callback for octomap updates
   * @param msg Octomap message containing the latest environment representation
   */
  void updateOctomap(const octomap_msgs::OctomapConstPtr &msg);
  
  /**
   * @brief Timer callback to periodically perform frontier exploration
   * @param event Timer event information
   */
  void performExploration(const ros::TimerEvent &event);

  /**
   * @brief Converts frontier points to a PCL point cloud
   * @return Pointer to a PCL point cloud containing frontier points
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr createFrontierPointCloud();
  
  /**
   * @brief Identifies the largest cluster of frontier points
   * @param cloud Point cloud containing all frontier points
   * @return Pointer to a PCL point cloud containing only the largest cluster
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr findLargestCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  
  /**
   * @brief Calculates the centroid of a point cloud to use as a goal point
   * @param cloud Point cloud to calculate the centroid for
   * @return The centroid point (average of all points in the cloud)
   */
  pcl::PointXYZ computeGoalCentroid(pcl::PointCloud<pcl::PointXYZ> &cloud);
  
  /**
   * @brief Detects frontier points in the current octomap
   *
   * Searches for points that are at the boundary between known free space and unknown space
   */
  void findFrontierPoints();
  
  /**
   * @brief Determines if a given point is a frontier point
   * @param coordinate The 3D coordinate to check
   * @return True if the point is a frontier point, false otherwise
   */
  bool isFrontierPoint(const octomap::point3d &coordinate);
  
  /**
   * @brief Publishes the calculated goal point for the drone to navigate to
   * @param goal_point The 3D point to publish as the next exploration goal
   */
  void publishExplorationGoal(const pcl::PointXYZ &goal_point);
};

DroneFrontierExploration::DroneFrontierExploration() {
  // Initialize subscribers and publishers
  position_subscriber_ = node_handle_.subscribe("true_body", 1, &DroneFrontierExploration::updateDronePosition, this);
  octomap_subscriber_ = node_handle_.subscribe("octomap_full", 1, &DroneFrontierExploration::updateOctomap, this);
  goal_publisher_ = node_handle_.advertise<geometry_msgs::Point>("/goal_position", 1);

  // Get parameters from the parameter server
  node_handle_.getParam("/octomap_server/resolution", octomap_resolution_);
  
  // Create timer for periodic exploration (runs at 0.3 Hz)
  exploration_timer_ = node_handle_.createTimer(ros::Rate(0.3), &DroneFrontierExploration::performExploration, this);
}

void DroneFrontierExploration::updateDronePosition(const nav_msgs::Odometry &msg) {
  // Update the current drone position from odometry message
  drone_position_ = octomap::point3d(
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      msg.pose.pose.position.z);
}

void DroneFrontierExploration::updateOctomap(const octomap_msgs::OctomapConstPtr &msg) {
  // Convert ROS octomap message to octomap::OcTree
  octomap_tree_ = std::make_shared<octomap::OcTree>(
      *dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg)));
}

void DroneFrontierExploration::performExploration(const ros::TimerEvent &event) {
  // Skip if octomap is not available yet
  if (!octomap_tree_)
    return;

  // Find frontier points in the current octomap
  findFrontierPoints();
  
  // Convert frontier points to PCL point cloud
  auto frontier_cloud = createFrontierPointCloud();

  // Cluster frontier points using OPTICS algorithm
  // Parameters: min points per cluster = 5, max reachability distance = 10.0
  Optics::optics<pcl::PointXYZ>(frontier_cloud, 5, 10.0, frontier_clusters_);

  // Find the largest cluster of frontier points
  auto largest_cluster = findLargestCluster(frontier_cloud);
  
  // Calculate the centroid of the largest cluster as the goal point
  auto goal_point = computeGoalCentroid(*largest_cluster);

  // Log the selected goal point
  ROS_INFO("Exploration goal point set to: (%f, %f, %f)",
           goal_point.x, goal_point.y, goal_point.z);
           
  // Publish the goal point for the drone to navigate to
  publishExplorationGoal(goal_point);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DroneFrontierExploration::createFrontierPointCloud() {
  // Create a new point cloud to store frontier points
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->header.frame_id = "world";
  cloud->is_dense = false;

  // Convert geometry_msgs::Point to pcl::PointXYZ
  for (const auto &point : frontier_points_) {
    cloud->points.emplace_back(point.x, point.y, point.z);
  }
  
  // Set point cloud dimensions
  cloud->width = cloud->points.size();
  cloud->height = 1;  // Unorganized point cloud

  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
DroneFrontierExploration::findLargestCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  // Find the cluster with the most points
  int max_cluster_size = 0;
  pcl::PointIndices::Ptr largest_cluster_indices;
  
  for (const auto &cluster : frontier_clusters_) {
    if (cluster->indices.size() > max_cluster_size) {
      largest_cluster_indices = cluster;
      max_cluster_size = cluster->indices.size();
    }
  }

  // Create a new point cloud containing only points from the largest cluster
  pcl::PointCloud<pcl::PointXYZ>::Ptr largest_cluster(
      new pcl::PointCloud<pcl::PointXYZ>);
      
  if (largest_cluster_indices) {
    for (int index : largest_cluster_indices->indices) {
      largest_cluster->points.push_back(cloud->points[index]);
    }
  }
  
  return largest_cluster;
}

pcl::PointXYZ DroneFrontierExploration::computeGoalCentroid(pcl::PointCloud<pcl::PointXYZ> &cloud) {
  // Initialize goal point at origin
  pcl::PointXYZ centroid(0, 0, 0);
  
  // Return origin if cloud is empty
  if (cloud.points.empty())
    return centroid;

  // Calculate sum of all points
  for (const auto &point : cloud.points) {
    centroid.x += point.x;
    centroid.y += point.y;
    centroid.z += point.z;
  }
  
  // Divide by number of points to get average (centroid)
  centroid.x /= cloud.points.size();
  centroid.y /= cloud.points.size();
  centroid.z /= cloud.points.size();

  return centroid;
}

void DroneFrontierExploration::findFrontierPoints() {
  // Clear previous frontier points
  frontier_points_.clear();
  
  // Define bounding box around current position to search for frontiers
  octomap::point3d min_point(
      drone_position_.x() - max_exploration_distance_,
      drone_position_.y() - max_exploration_distance_,
      drone_position_.z() - max_exploration_distance_);
      
  octomap::point3d max_point(
      std::min(drone_position_.x() + max_exploration_distance_, -340.0f),
      drone_position_.y() + max_exploration_distance_,
      drone_position_.z() + max_exploration_distance_);
      
  // Get iterator for leaf nodes in the bounding box
  octomap_iterator_ = octomap_tree_->begin_leafs_bbx(min_point, max_point);
  
  // Iterate through all leaf nodes in the bounding box
  for (; octomap_iterator_ != octomap_tree_->end_leafs_bbx(); ++octomap_iterator_) {
    octomap::point3d coordinate = octomap_iterator_.getCoordinate();
    
    // Check if the node is free (not occupied)
    if (!octomap_tree_->isNodeOccupied(*octomap_iterator_)) {
      // Check if it's a frontier point (has unknown neighbors)
      if (isFrontierPoint(coordinate)) {
        // Add to frontier points list
        geometry_msgs::Point frontier_point;
        frontier_point.x = coordinate.x();
        frontier_point.y = coordinate.y();
        frontier_point.z = coordinate.z();
        frontier_points_.push_back(frontier_point);
      }
    }
  }
}

bool DroneFrontierExploration::isFrontierPoint(const octomap::point3d &coordinate) {
  // Define the search offsets to check the immediate neighbors in 3D,
  // including diagonals
  std::vector<octomap::point3d> neighbor_offsets = {
      // 6 face neighbors
      {1, 0, 0},   {-1, 0, 0},   {0, 1, 0},
      {0, -1, 0},  {0, 0, 1},    {0, 0, -1},
      
      // 12 edge neighbors
      {1, 1, 0},   {-1, -1, 0},  {1, -1, 0}, {-1, 1, 0}, // XY plane diagonals
      {1, 0, 1},   {-1, 0, -1},  {0, 1, 1}, {0, -1, -1}, // XZ and YZ plane diagonals
      {-1, 0, 1},  {1, 0, -1},   {0, -1, 1}, {0, 1, -1}, // Opposite diagonals
      
      // 8 corner neighbors
      {1, 1, 1},   {-1, -1, -1}, {1, -1, 1}, {-1, 1, -1}, // 3D diagonals
      {1, 1, -1},  {-1, -1, 1},  {1, -1, -1}, {-1, 1, 1}
  };

  // Check all neighboring cells
  for (const auto &offset : neighbor_offsets) {
    // Calculate neighbor coordinate adjusted by octomap resolution
    octomap::point3d neighbor_coordinate =
        coordinate + offset * octomap_resolution_;
        
    // Search for the node in the octomap
    octomap::OcTreeNode *node = octomap_tree_->search(neighbor_coordinate);
    
    // If the node doesn't exist (is unknown space), this is a frontier point
    if (!node) {
      return true;
    }
  }
  
  // If all neighbors are known (either occupied or free), it's not a frontier
  return false;
}

void DroneFrontierExploration::publishExplorationGoal(const pcl::PointXYZ &goal_point) {
  // Convert PCL point to ROS message
  geometry_msgs::Point goal_msg;
  goal_msg.x = goal_point.x;
  goal_msg.y = goal_point.y;
  goal_msg.z = goal_point.z;

  // TODO: Use mode_switch here later
  if (goal_msg.x <= -340.0) {
    goal_publisher_.publish(goal_msg);
  }
}

int main(int argc, char **argv) {
  // Initialize ROS node
  ros::init(argc, argv, "drone_frontier_exploration");
  
  // Create exploration object
  DroneFrontierExploration frontier_exploration;

  // Enter ROS event loop
  ros::spin();
  return 0;
}
