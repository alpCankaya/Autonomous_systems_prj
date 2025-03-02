#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from tf.transformations import quaternion_from_euler
import time
import sys
import os

# Import D* Lite
sys.path.append("/home/rehametu/src/dstar_explorer/scripts")  
from dstar_lite import DStarLite

class DStarExplorer:
    def __init__(self):
        rospy.init_node('dstar_explorer')

        # Subscribers
        self.frontier_sub = rospy.Subscriber('/frontier_goal', PoseStamped, self.frontier_callback)
        self.true_pose_sub = rospy.Subscriber("/true_pose", PoseStamped, self.true_pose_callback)

        # Publishers
        self.trajectory_pub = rospy.Publisher('/desired_state', MultiDOFJointTrajectory, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_position_pub = rospy.Publisher('/goal_position', PoseStamped, queue_size=10)

        # D* Lite Planner
        self.dstar = DStarLite()

        # State Variables
        self.current_goal = None
        self.body_position = None
        self.body_orientation = None
        self.last_rotation_time = time.time()

    def true_pose_callback(self, msg):
        """ Updates the drone's position and orientation. """
        self.body_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.body_orientation = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        rospy.loginfo(f"📍 Updated drone position: {self.body_position}")

    def frontier_callback(self, msg):
        """ Handles new frontier goals. """
        self.current_goal = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        rospy.loginfo(f"🎯 New frontier received: {self.current_goal}")

        # Publish absolute goal position
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = self.current_goal[0]
        goal_msg.pose.position.y = self.current_goal[1]
        goal_msg.pose.position.z = self.current_goal[2]
        goal_msg.pose.orientation.w = 1.0  # Default orientation

        self.goal_position_pub.publish(goal_msg)
        rospy.loginfo(f"📡 Published goal position to /goal_position: {goal_msg.pose.position.x}, {goal_msg.pose.position.y}, {goal_msg.pose.position.z}")

        self.plan_path()

    def plan_path(self):
        """ Plans a path using D* Lite. """
        if self.current_goal is None or self.body_position is None:
            rospy.logwarn("❌ Missing current goal or robot position for D* planning.")
            return

        rospy.loginfo(f"🚀 Planning path from {self.body_position} to {self.current_goal}")
        path_msg = self.dstar.plan(tuple(self.body_position[:2]), tuple(self.current_goal[:2]))

        if path_msg is None or not path_msg.poses:
            rospy.logwarn("⚠️ D* Lite failed to find a valid path.")
            return

        self.publish_trajectory(path_msg)

    def publish_trajectory(self, path_msg):
        """ Publishes planned trajectory and velocity commands. """
        if not path_msg.poses:
            rospy.logwarn("🚨 Received empty path, skipping trajectory publication.")
            return

        trajectory = MultiDOFJointTrajectory()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.header.frame_id = "map"

        twist_msg = Twist()

        for i in range(len(path_msg.poses) - 1):
            current_pose = path_msg.poses[i].pose
            next_pose = path_msg.poses[i + 1].pose

            # MultiDOFJointTrajectoryPoint
            traj_point = MultiDOFJointTrajectoryPoint()

            # Position
            traj_point.transforms.append(PoseStamped().pose)
            traj_point.transforms[0].position.x = current_pose.position.x
            traj_point.transforms[0].position.y = current_pose.position.y
            traj_point.transforms[0].position.z = current_pose.position.z

            # Orientation (Yaw Rotation)
            yaw = np.arctan2(next_pose.position.y - current_pose.position.y,
                             next_pose.position.x - current_pose.position.x)
            quat = quaternion_from_euler(0, 0, yaw)
            traj_point.transforms[0].orientation = Quaternion(*quat)

            # Velocity Calculation
            dt = 1.0  # Fixed time step
            vel_x = (next_pose.position.x - current_pose.position.x) / dt
            vel_y = (next_pose.position.y - current_pose.position.y) / dt
            vel_z = (next_pose.position.z - current_pose.position.z) / dt

            traj_point.velocities.append(Twist().linear)
            traj_point.velocities[0].x = vel_x
            traj_point.velocities[0].y = vel_y
            traj_point.velocities[0].z = vel_z

            trajectory.points.append(traj_point)

            # Set velocity for cmd_vel
            twist_msg.linear.x = vel_x
            twist_msg.linear.y = vel_y
            twist_msg.angular.z = yaw * 0.5  # Angular velocity scaling

        if trajectory.points:
            self.trajectory_pub.publish(trajectory)
            self.cmd_vel_pub.publish(twist_msg)
            rospy.loginfo(f"🚀 Published trajectory and cmd_vel: {twist_msg.linear.x}, {twist_msg.linear.y}, {twist_msg.angular.z}")
        else:
            rospy.logwarn("⚠️ No trajectory points generated, skipping publish.")

if __name__ == '__main__':
    explorer = DStarExplorer()
    rospy.spin()

