#!/usr/bin/env python3

import rospy
import numpy as np
from octomap_msgs.msg import Octomap
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
import cv2  # Used for connected components (frontier grouping)

class FrontierDetector:
    def __init__(self):
        rospy.init_node('frontier_detector')

        # Subscribers
        self.octomap_sub = rospy.Subscriber('/octomap_binary', Octomap, self.map_callback)
        self.true_pose_sub = rospy.Subscriber("/true_pose", PoseStamped, self.pose_callback)

        # Publishers
        self.frontier_pub = rospy.Publisher('/frontier_goal', PoseStamped, queue_size=10)
        self.frontiers_viz_pub = rospy.Publisher('/frontiers_grid', OccupancyGrid, queue_size=10)
	self.frontier_marker_pub = rospy.Publisher('/frontier_markers', MarkerArray, queue_size=10)

        # State
        self.occupancy_grid = None
        self.resolution = 0.2  # Set based on OctoMap resolution
        self.origin = [0, 0]  # World frame origin
        self.drone_position = None

    def pose_callback(self, msg):
        """Updates the drone's world position."""
        self.drone_position = np.array([msg.pose.position.x, msg.pose.position.y])

    def map_callback(self, msg):
        """Processes the OctoMap, extracts frontiers, and publishes the best one."""
        self.occupancy_grid = self.convert_octomap_to_grid(msg)
        frontiers = self.detect_frontiers()

        if frontiers:
            best_frontier = self.select_best_frontier(frontiers)
            self.publish_frontier_goal(best_frontier)

    def convert_octomap_to_grid(self, octomap_msg):
        """Converts 3D OctoMap to a 2D occupancy grid."""
        grid_size = (200, 200)  # Adjust depending on your map size
        occupancy_grid = np.full(grid_size, -1, dtype=np.int8)  # Unknown (-1)

        for node in octomap_msg.data:
            x, y = int(node.x / self.resolution), int(node.y / self.resolution)
            if node.occupied:
                occupancy_grid[y, x] = 100  # Occupied
            else:
                occupancy_grid[y, x] = 0  # Free space

        return occupancy_grid

    def detect_frontiers(self):
        """Finds frontier cells in the occupancy grid."""
        if self.occupancy_grid is None:
            rospy.logwarn("No occupancy grid available for frontier detection.")
            return []

        frontier_mask = np.zeros_like(self.occupancy_grid, dtype=np.uint8)

        for y in range(1, self.occupancy_grid.shape[0] - 1):
            for x in range(1, self.occupancy_grid.shape[1] - 1):
                if self.occupancy_grid[y, x] == 0:  # Free space
                    # Check if any neighbor is unknown (-1)
                    if np.any(self.occupancy_grid[y-1:y+2, x-1:x+2] == -1):
                        frontier_mask[y, x] = 255  # Mark as frontier

        # Group frontier pixels into clusters using OpenCV
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(frontier_mask, connectivity=8)

        # Extract frontier centroids
        frontiers = []
        for i in range(1, num_labels):  # Ignore background label 0
            x, y = stats[i, cv2.CC_STAT_LEFT] + stats[i, cv2.CC_STAT_WIDTH] // 2, \
                   stats[i, cv2.CC_STAT_TOP] + stats[i, cv2.CC_STAT_HEIGHT] // 2
            frontiers.append((x, y))

	self.publish_frontier_markers(frontiers)
        return frontiers

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
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = frontier[0] * self.resolution
        goal.pose.position.y = frontier[1] * self.resolution
        goal.pose.position.z = 1.0  # Keep the drone at constant height

        self.frontier_pub.publish(goal)
        rospy.loginfo(f"Published new frontier goal: {goal.pose.position.x}, {goal.pose.position.y}")
	
    def publish_frontier_markers(self, frontiers):
	"""Publishes frontiers as visualization markers in RViz."""
	marker_array = MarkerArray()

	for i, (x, y) in enumerate(frontiers):
		marker = Marker()
		marker.header.frame_id = "map"  # Change if using another frame
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

