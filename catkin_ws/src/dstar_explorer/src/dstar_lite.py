class DStarLite:
    def __init__(self):
        print("[D* Lite] Initialized D* Lite Planner.")

    def plan(self, goal):
        """Generate a simple straight-line path to the goal (for testing)."""
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped
        import rospy

        path = Path()
        path.header.frame_id = "world"

        for i in range(10):  # Generate 10 waypoints
            pose = PoseStamped()
            pose.pose.position.x = goal[0] + i * 0.5  # Move towards goal
            pose.pose.position.y = goal[1]
            pose.pose.position.z = goal[2]
            path.poses.append(pose)

        return path
