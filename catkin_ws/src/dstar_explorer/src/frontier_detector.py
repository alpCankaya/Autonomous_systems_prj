#!/usr/bin/env python3

import rospy
import numpy as np
from octomap_msgs.msg import Octomap
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
import cv2  # Used for connected components (frontier grouping)

def find_frontiers(octomap_msg):
    """Finds frontiers from an OctoMap message and returns a list of unexplored areas."""
    
    if not hasattr(octomap_msg, 'data') or not octomap_msg.data:
        rospy.logwarn("⚠️ Received empty or invalid OctoMap message.")
        return []

    grid_size = (200, 200)  # Grid dimensions
    resolution = 0.2  # Resolution of OctoMap
    occupancy_grid = np.full(grid_size, -1, dtype=np.int8)  # Unknown (-1)

    rospy.loginfo("✅ Converting OctoMap data to occupancy grid...")

    # Convert OctoMap binary data into a 2D occupancy grid
    for idx, value in enumerate(octomap_msg.data):
        x = (idx % grid_size[0]) * resolution
        y = (idx // grid_size[1]) * resolution

        grid_x = min(max(int(x / resolution), 0), grid_size[0] - 1)
        grid_y = min(max(int(y / resolution), 0), grid_size[1] - 1)

        if value > 0:  # Occupied space
            occupancy_grid[grid_y, grid_x] = 100
        elif value == 0:  # Free space
            occupancy_grid[grid_y, grid_x] = 0

    rospy.loginfo("✅ Identifying frontiers...")

    # Identify frontier regions
    frontier_mask = np.zeros_like(occupancy_grid, dtype=np.uint8)

    for y in range(1, occupancy_grid.shape[0] - 1):
        for x in range(1, occupancy_grid.shape[1] - 1):
            if occupancy_grid[y, x] == 0:  # Free space
                if np.any(occupancy_grid[y-1:y+2, x-1:x+2] == -1):  # Check for unknown neighbors
                    frontier_mask[y, x] = 255  # Mark as frontier

    rospy.loginfo("✅ Clustering frontier points...")

    # Group frontier pixels into clusters using OpenCV
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(frontier_mask, connectivity=8)

    frontiers = []
    for i in range(1, num_labels):  # Ignore background label 0
        x = stats[i, cv2.CC_STAT_LEFT] + stats[i, cv2.CC_STAT_WIDTH] // 2
        y = stats[i, cv2.CC_STAT_TOP] + stats[i, cv2.CC_STAT_HEIGHT] // 2

        if 0 <= x < grid_size[0] and 0 <= y < grid_size[1]:  # Ensure valid indices
            frontiers.append((x * resolution, y * resolution))

    rospy.loginfo(f"✅ DEBUG: Detected {len(frontiers)} frontiers -> {frontiers}")

    return frontiers


class FrontierDetector:
    def __init__(self):
        rospy.init_node('frontier_detector')

        # Subscribers
        self.octomap_sub = rospy.Subscriber('/octomap_binary', Octomap, self.map_callback)
        self.current_state_sub = rospy.Subscriber("/current_state_est", Odometry, self.current_state_callback)

        # Publishers
        self.frontier_pub = rospy.Publisher('/frontier_goal', PoseStamped, queue_size=10)
        self.frontiers_viz_pub = rospy.Publisher('/frontiers_grid', OccupancyGrid, queue_size=10)
        self.frontier_marker_pub = rospy.Publisher('/frontier_markers', MarkerArray, queue_size=10)

        # State
        self.occupancy_grid = None
        self.resolution = 0.2  # Set based on OctoMap resolution
        self.origin = [0, 0]  # World frame origin
        self.drone_position = None

    def current_state_callback(self, msg):
        """Updates the drone's world position from Odometry data."""
        self.drone_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

    def map_callback(self, msg):
        """Processes the OctoMap, extracts frontiers, and publishes the best one."""
        frontiers = find_frontiers(msg)

        if frontiers:
            best_frontier = self.select_best_frontier(frontiers)
            self.publish_frontier_goal(best_frontier)
            self.publish_frontier_markers(frontiers)

    def select_best_frontier(self, frontiers):
        """Selects the best frontier based on distance to the drone."""
        if self.drone_position is None:
            rospy.logwarn("Drone position not available, selecting a random frontier.")
            return frontiers[0]  # Default to first found frontier

        # Sort frontiers by Euclidean distance
        frontiers.sort(key=lambda f: np.linalg.norm(np.array(f) - self.drone_position))
        return frontiers[0]

    def publish_frontier_goal(self, frontier):
        """Publishes the selected frontier as the next goal for D* planning."""
        rospy.loginfo(f"✅ DEBUG: Attempting to publish frontier goal at ({frontier[0]}, {frontier[1]})")

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = frontier[0]
        goal.pose.position.y = frontier[1]
        goal.pose.position.z = 1.0  # Keep the drone at a constant height

        self.frontier_pub.publish(goal)
        rospy.loginfo(f"✅ DEBUG: Successfully published frontier goal at ({goal.pose.position.x}, {goal.pose.position.y})")

    def publish_frontier_markers(self, frontiers):
        """Publishes frontiers as visualization markers in RViz."""
        marker_array = MarkerArray()

        for i, (x, y) in enumerate(frontiers):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "frontiers"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x * self.resolution
            marker.pose.position.y = y * self.resolution
            marker.pose.position.z = 1.0  # Keep at a fixed height
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Fully visible

            marker_array.markers.append(marker)

        self.frontier_marker_pub.publish(marker_array)
        rospy.loginfo(f"Published {len(frontiers)} frontier markers in RViz.")


if __name__ == '__main__':
    detector = FrontierDetector()
    rospy.spin()
