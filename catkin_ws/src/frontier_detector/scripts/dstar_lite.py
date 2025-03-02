#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class DStarLite:
    def __init__(self):
        rospy.loginfo("🔄 D* Lite Path Planner Initialized!")

    def plan(self, start, goal):
        """ D* Lite algoritması ile başlangıçtan hedefe yol planlar. """
        rospy.loginfo(f"📍 Planning path from {start} to {goal}")

        if np.linalg.norm(np.array(start) - np.array(goal)) < 0.5:
            rospy.logwarn("🚀 Already at goal! No need to plan a new path.")
            return None

        # Örnek olarak sadece bir düz yol oluşturalım
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        step_size = 1.0  # 1 metre adımlarla ilerleyelim
        direction = np.array(goal) - np.array(start)
        direction = direction / np.linalg.norm(direction)  # Birim vektör yap

        current_position = np.array(start)
        waypoints = []

        while np.linalg.norm(current_position - np.array(goal)) > step_size:
            current_position += direction * step_size

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = current_position[0]
            pose.pose.position.y = current_position[1]
            pose.pose.position.z = 1.0  # Sabit yükseklikte kalsın

            waypoints.append(pose)

        if not waypoints:
            rospy.logwarn("❌ No valid path found!")
            return None

        path_msg.poses = waypoints
        rospy.loginfo(f"✅ Path planned with {len(waypoints)} waypoints.")
        return path_msg

