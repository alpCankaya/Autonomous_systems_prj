#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Path, Odometry
from octomap_msgs.msg import Octomap
from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from tf.transformations import quaternion_from_euler, quaternion_matrix
from std_msgs.msg import Bool
from trajectory_msgs.msg import MultiDOFJointTrajectory
from geometry_msgs.msg import Transform

import random
import time
import sys
import os

# Add the src directory to Python's path
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../src")

from dstar_lite import DStarLite
from frontier_detector import find_frontiers


class DStarExplorer:
    def __init__(self):
        rospy.init_node('dstar_explorer')

        # Subscribers
        self.octomap_sub = rospy.Subscriber('/octomap_binary', Octomap, self.map_callback)
        self.frontier_sub = rospy.Subscriber('/frontier_goal', PoseStamped, self.frontier_callback)
        self.current_state_sub = rospy.Subscriber("/current_state_est", Odometry, self.current_state_callback)
        self.lantern_detected_sub = rospy.Subscriber('/lantern_detected', Bool, self.lantern_callback)

        # Path publisher
        self.path_pub = rospy.Publisher('/dstar_path', Path, queue_size=10)

        # Publishers
        self.trajectory_pub = rospy.Publisher('/desired_state', MultiDOFJointTrajectoryPoint, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # D* Lite Planner (Assuming it's implemented)
        self.dstar = DStarLite()

        # State Variables
        self.frontier_list = []  # List of unexplored frontiers
        self.current_goal = None
        self.body_position = None
        self.body_orientation = None
        self.lantern_detected = False
        self.last_rotation_time = time.time()

    def current_state_callback(self, msg):
        """Updates the true global position and orientation of the drone using /current_state_est."""
        self.body_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        self.body_orientation = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w  # Quaternion w component
        ])

        # Convert quaternion to rotation matrix (3x3)
        self.rotation_matrix = quaternion_matrix(self.body_orientation)[:3, :3]

        rospy.loginfo(f"✅ Updated body position: {self.body_position}")
        rospy.loginfo(f"✅ Updated body orientation (quaternion): {self.body_orientation}")
        rospy.loginfo(f"✅ Computed rotation matrix:\n{self.rotation_matrix}")

    def lantern_callback(self, msg):
        """Handles detected lanterns by stopping and recording their positions."""
        if msg.data:  # Lantern detected
            rospy.loginfo("Lantern detected! Stopping exploration momentarily.")
            self.lantern_detected = True
            rospy.sleep(3)  # Pause to record position
            self.lantern_detected = False  # Resume exploration

    def map_callback(self, msg):
        """Processes OctoMap data to find new frontiers and replan paths dynamically."""
        self.update_frontiers(msg)
        
        if not self.lantern_detected:
            if self.frontier_list:
                self.current_goal = self.select_best_frontier()
                self.plan_path()
            else:
                self.rotate_for_better_visibility()

    def update_frontiers(self, octomap):
        """Extracts unexplored frontiers from OctoMap data."""
        self.frontier_list = find_frontiers(octomap)

    def select_best_frontier(self):
        """Selects the best unexplored frontier using heuristic selection."""
        if self.frontier_list:
            return random.choice(self.frontier_list)  # Select a random unexplored point
        return None

    def plan_path(self):
        """Plans a path using D* Lite and publishes it as a trajectory."""
        if self.current_goal:
            rospy.loginfo(f"Planning path to frontier: {self.current_goal}")

            # Ensure current_goal is always (x, y, z)
            if len(self.current_goal) == 2:
                self.current_goal = (self.current_goal[0], self.current_goal[1], 1.0)

            rospy.loginfo(f"✅ DEBUG: Fixed self.current_goal = {self.current_goal} (Tuple)")

            path_msg = self.dstar.plan(self.current_goal)
            
            # Convert to trajectory format
            trajectory_msg = self.convert_path_to_trajectory(path_msg)
            
            # Publish the trajectory message
            for point in trajectory_msg:
                self.trajectory_pub.publish(point)  # ✅ Publish each point separately

            rospy.loginfo("✅ Published trajectory to /desired_trajectory")

    def convert_path_to_trajectory(self, path_msg):
        """Converts a nav_msgs/Path to a MultiDOFJointTrajectory with global coordinates and rotation applied."""

        if self.body_position is None or self.rotation_matrix is None:
            rospy.logwarn("⚠️ Warning: body_position or rotation_matrix not set! Using default values.")
            self.body_position = np.array([-320.0, 15.0, 17.0])  # Default to cave entrance
            self.rotation_matrix = np.eye(3)  # Default to identity matrix (no rotation)

        trajectory_points = []
        dt = 1.5  # Slower trajectory

        for i in range(len(path_msg.poses) - 1):
            point = MultiDOFJointTrajectoryPoint()
            transform = Transform()

            local_position = np.array([
                path_msg.poses[i].pose.position.x,
                path_msg.poses[i].pose.position.y,
                path_msg.poses[i].pose.position.z
            ])

            global_position = np.dot(self.rotation_matrix, local_position) + self.body_position
            global_position[2] = self.apply_z_constraints(global_position[2])

            transform.translation.x = global_position[0]
            transform.translation.y = global_position[1]
            transform.translation.z = global_position[2]

            next_local_position = np.array([
                path_msg.poses[i + 1].pose.position.x,
                path_msg.poses[i + 1].pose.position.y,
                path_msg.poses[i + 1].pose.position.z
            ])
            global_next_position = np.dot(self.rotation_matrix, next_local_position) + self.body_position

            vel_x = (global_next_position[0] - global_position[0]) / dt
            vel_y = (global_next_position[1] - global_position[1]) / dt
            vel_z = (global_next_position[2] - global_position[2]) / dt

            velocity = Twist()
            velocity.linear.x = vel_x
            velocity.linear.y = vel_y
            velocity.linear.z = vel_z

            acceleration = Twist()
            acceleration.linear.x = 0.0
            acceleration.linear.y = 0.0
            acceleration.linear.z = 0.0

            yaw = np.arctan2(vel_y, vel_x)
            quat = quaternion_from_euler(0, 0, yaw)
            transform.rotation = Quaternion(*quat)

            point.transforms = [transform]
            point.velocities = [velocity]
            point.accelerations = [acceleration]
            point.time_from_start = rospy.Duration(i * dt)

            trajectory_points.append(point)

        return trajectory_points

    def apply_z_constraints(self, proposed_z):
        min_z, max_z = 12.0, 18.0
        return max(min_z, min(proposed_z, max_z))

    def frontier_callback(self, msg):
        self.current_goal = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z if msg.pose.position.z else 1.0)
        rospy.loginfo(f"✅ DEBUG: Updated current goal to {self.current_goal}")
        self.plan_path()


if __name__ == '__main__':
    explorer = DStarExplorer()
    rospy.spin()
