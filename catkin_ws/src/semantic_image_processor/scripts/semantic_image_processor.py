#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ObjectDepthEstimator:
    def __init__(self):
        rospy.init_node('object_depth_estimator', anonymous=True)
        
        self.bridge = CvBridge()
        self.semantic_image = None
        self.depth_image = None
        self.step_size = 5  # Reduce for faster processing
        
        # Subscribe to semantic and depth image topics
        rospy.Subscriber("/unity_ros/Quadrotor/Sensors/SemanticCamera/image_raw", Image, self.semantic_callback)
        rospy.Subscriber("/realsense/depth/image", Image, self.depth_callback)

    def semantic_callback(self, msg):
        """Processes the semantic image and computes the average depth of non-black pixels."""
        self.semantic_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Find non-black pixels
        non_black_mask = np.any(self.semantic_image != [0, 0, 0], axis=-1)
        non_black_pixels = np.column_stack(np.where(non_black_mask))  # (v, u) format

        if non_black_pixels.size > 0:  # Check if any valid pixels exist
            avg_v = np.mean(non_black_pixels[:, 0])  # y-coordinates (rows)
            avg_u = np.mean(non_black_pixels[:, 1])  # x-coordinates (cols)
            
            rospy.loginfo(f"Avg pixel (u, v) = ({avg_u:.2f}, {avg_v:.2f})")

            if self.depth_image is not None:
                self.get_average_depth(non_black_pixels)
    
    def depth_callback(self, msg):
        """Stores the latest depth image."""
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def get_average_depth(self, pixel_list):
        """Retrieves the depth values at all given pixel coordinates and averages them."""
        if self.depth_image is None:
            rospy.logwarn("No depth image received yet!")
            return

        depth_values = []
        camera_coordinates = []  # To store 3D camera coordinates
        height, width = self.depth_image.shape

        for v, u in pixel_list:
            if 0 <= u < width and 0 <= v < height:
                depth_value = self.depth_image[int(v), int(u)] / 1000.0  # Convert mm to meters
                if depth_value > 0:  # Ignore invalid depth (0 values)
                    depth_values.append(depth_value)

                    # Convert pixel coordinates to camera coordinates (corrected for Y, X, Z)
                    X = (u - 160) * depth_value / 120  # Right (X)
                    Y = (v - 120) * depth_value / 120  # Up (Z)
                    Z = depth_value  # Depth (Y)
                    camera_coordinates.append((X, Z, Y))

        if depth_values:
            avg_depth = np.mean(depth_values)
            rospy.loginfo(f"Avg depth of object: {avg_depth:.2f} meters")
            
            # Log camera coordinates for the object
            for coord in camera_coordinates:
                rospy.loginfo(f"Camera coordinates: {coord}")

        else:
            rospy.logwarn("No valid depth values found!")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    estimator = ObjectDepthEstimator()
    estimator.run()
