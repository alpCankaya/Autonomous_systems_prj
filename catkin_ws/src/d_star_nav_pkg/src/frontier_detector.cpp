#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <cmath>

class FrontierDetector {
public:
  FrontierDetector() {
    ros::NodeHandle nh;
    // Subscribe to the occupancy grid published on "world"
    map_sub_ = nh.subscribe("/projected_map", 1, &FrontierDetector::mapCallback, this);
    // Publish detected frontiers as PoseArray
    frontier_pub_ = nh.advertise<geometry_msgs::PoseArray>("frontiers", 1);
  }
  
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    nav_msgs::OccupancyGrid map = *msg;
    geometry_msgs::PoseArray frontier_poses;
    frontier_poses.header = map.header;
    
    int width = map.info.width;
    int height = map.info.height;
    double resolution = map.info.resolution;
    double origin_x = map.info.origin.position.x;
    double origin_y = map.info.origin.position.y;
    
    // Loop through all cells
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        int idx = x + y * width;
        int occ = map.data[idx];
        // Treat free cells (value 0) as candidates
        if (occ == 0) {
          bool is_frontier = false;
          // Check 8-connected neighbors for an unknown cell (-1)
          for (int dy = -1; dy <= 1 && !is_frontier; dy++) {
            for (int dx = -1; dx <= 1 && !is_frontier; dx++) {
              int nx = x + dx;
              int ny = y + dy;
              if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                continue;
              int nidx = nx + ny * width;
              if (map.data[nidx] == -1) {  // unknown cell found
                is_frontier = true;
              }
            }
          }
          if (is_frontier) {
            geometry_msgs::Pose pose;
            // Convert grid cell to world coordinates (center of cell)
            pose.position.x = origin_x + (x + 0.5) * resolution;
            pose.position.y = origin_y + (y + 0.5) * resolution;
            pose.position.z = 0;  // for 2D map (set z as needed)
            frontier_poses.poses.push_back(pose);
          }
        }
      }
    }
    frontier_pub_.publish(frontier_poses);
    ROS_INFO("Published %lu frontier points", frontier_poses.poses.size());
  }
  
private:
  ros::Subscriber map_sub_;
  ros::Publisher frontier_pub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "frontier_detector");
  FrontierDetector detector;
  ros::spin();
  return 0;
}
