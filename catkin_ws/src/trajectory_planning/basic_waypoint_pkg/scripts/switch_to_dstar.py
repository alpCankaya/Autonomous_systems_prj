#!/usr/bin/env python3

import rospy
import subprocess
import numpy as np
from geometry_msgs.msg import PoseStamped  # Changed from Odometry

class GoalMonitor:
    def __init__(self):
        rospy.init_node('goal_monitor')

        # Subscribe to true UAV position (changed from /odom to /true_pose)
        rospy.Subscriber('/true_pose', PoseStamped, self.pose_callback)

        # Define the cave entrance goal position (same as planner_node)
        self.goal_position = np.array([-320.0, 15.0, 17.0])
        self.reached_goal = False

        rospy.loginfo("Goal monitor started. Waiting for UAV to reach the cave entrance...")

    def pose_callback(self, msg):
        """Checks if UAV reached the goal position."""
        drone_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        distance = np.linalg.norm(self.goal_position - drone_position)

        rospy.loginfo(f"[Goal Monitor] UAV Position: {drone_position} | Distance to goal: {distance}")

        if distance < 1.5 and not self.reached_goal:  # Threshold: 1.5m
            rospy.loginfo("[Goal Monitor] Goal position reached! Switching to D* exploration.")
            self.reached_goal = True

            # Stop the planner node
            self.stop_planner()

            # Start the D* Explorer node
            self.start_dstar_explorer()

    def stop_planner(self):
        """Stops the predefined path planner node."""
        rospy.loginfo("Shutting down planner_node...")
        subprocess.call(["rosnode", "kill", "/planner"])

    def start_dstar_explorer(self):
        """Launches the D* exploration node."""
        rospy.loginfo("Launching D* exploration node...")
        subprocess.Popen(["rosrun", "dstar_explorer", "dstar_explorer.py"])
        subprocess.Popen(["rosrun", "dstar_explorer", "frontier_detector.py"])
if __name__ == '__main__':
    GoalMonitor()
    rospy.spin()
