#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_matrix

class ObjectDepthEstimator:
    def __init__(self):
        rospy.init_node('object_depth_estimator', anonymous=True)
        
        self.bridge = CvBridge()
        self.semantic_image = None
        self.depth_image = None
        
        # Subscribe to semantic and depth image topics
        rospy.Subscriber("/unity_ros/Quadrotor/Sensors/SemanticCamera/image_raw", Image, self.semantic_callback)
        rospy.Subscriber("/realsense/depth/image", Image, self.depth_callback)
        rospy.Subscriber("/true_pose", PoseStamped, self.true_pose_callback)  # Fixed to PoseStamped
        
        self.body_position = None  # (x, y, z) in world frame
        self.body_orientation = None  # Quaternion (x, y, z, w)
    
    def true_pose_callback(self, msg):
        """Updates the true pose (position & orientation) of the body in the world frame."""
        pose = msg.pose  # Extract Pose from PoseStamped
        
        self.body_position = np.array([pose.position.x, pose.position.y, pose.position.z])
        self.body_orientation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        
        #rospy.loginfo(f"Received Pose - Position: {self.body_position}, Orientation: {self.body_orientation}")

        # Fix quaternion sign issue
        if self.body_orientation[3] < 0:
            self.body_orientation *= -1
            #rospy.loginfo("Neg. w detected: Quaternion flipped for consistency.")
    
    def semantic_callback(self, msg):
        """Processes the semantic image and computes the average depth of non-black pixels."""
        self.semantic_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        non_black_mask = np.any(self.semantic_image != [0, 0, 0], axis=-1)
        non_black_pixels = np.column_stack(np.where(non_black_mask))

        if non_black_pixels.size > 0:
            selected_pixels = non_black_pixels[np.random.choice(non_black_pixels.shape[0], min(1000, non_black_pixels.shape[0]), replace=False)]
            avg_v = np.mean(selected_pixels[:, 0])
            avg_u = np.mean(selected_pixels[:, 1])
            rospy.loginfo(f"Avg pixel (u, v) = ({avg_u:.2f}, {avg_v:.2f})")

            if self.depth_image is not None:
                self.get_average_depth(selected_pixels)
    
    def depth_callback(self, msg):
        """Stores the latest depth image."""
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    
    def get_average_depth(self, selected_pixels):
        """Retrieves the average depth value of 100 randomly selected non-black pixels."""
        if self.depth_image is None:
            rospy.logwarn("No depth image received yet!")
            return

        valid_depths = []
        for v, u in selected_pixels:
            depth = self.depth_image[int(v), int(u)] / 1000.0  # Convert mm to meters
            if depth > 0:
                valid_depths.append(depth)
        
        if valid_depths:
            avg_depth = np.mean(valid_depths)
            
            X = avg_depth  # Depth (Y)
            Y = -(np.mean(selected_pixels[:, 1]) - 160) * avg_depth / 120 
            Z = -(np.mean(selected_pixels[:, 0]) - 120) * avg_depth / 120

            object_body_frame = np.array([X, Y, Z])
            rospy.loginfo(f"Object 3D position in body frame: ({X:.2f}, {Y:.2f}, {Z:.2f})")
            
            if self.body_position is not None and self.body_orientation is not None:
                self.transform_to_world(object_body_frame)
    
    def transform_to_world(self, object_body_frame):
        """Transforms the object's position from the body frame to the world frame."""
        R = quaternion_matrix(self.body_orientation)[:3, :3]  # Extract 3x3 rotation matrix
        object_world_frame = np.dot(R, object_body_frame) + self.body_position

        rospy.loginfo(f"Object 3D position in world frame: ({object_world_frame[0]:.2f}, {object_world_frame[1]:.2f}, {object_world_frame[2]:.2f})")
    
    def run(self):
        rospy.spin()

if __name__ == "__main__":
    estimator = ObjectDepthEstimator()
    estimator.run()
