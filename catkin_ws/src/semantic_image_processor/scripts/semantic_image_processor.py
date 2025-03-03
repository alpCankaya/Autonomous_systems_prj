#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_matrix
import os
from std_msgs.msg import Int32

class ObjectDetection:
    def __init__(self):
        rospy.init_node('object_detection')
        
        self.bridge = CvBridge()
        self.semantic_image = None
        self.depth_image = None
        self.body_position = None  
        self.body_orientation = None  
        # Dictionary to track detected objects.
        # Keys are unique object IDs and values are lists of recent detections.
        self.tracked_objects = {}
        self.next_object_id = 0  # Unique ID counter for objects

        # Set to keep track of objects already logged as stable.
        self.logged_objects = set()

        # File to save stable object positions
        self.output_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), "object_positions.txt")

        # Optionally, clear the file at startup
        open(self.output_file, "w").close()

        # Publisher for detected object count
        self.object_count_pub = rospy.Publisher("/detected_objects_count", Int32, queue_size=10)

        # Subscribe to topics
        rospy.Subscriber("/unity_ros/Quadrotor/Sensors/SemanticCamera/image_raw", Image, self.semantic_callback)
        rospy.Subscriber("/realsense/depth/image", Image, self.depth_callback)
        rospy.Subscriber("/true_pose", PoseStamped, self.true_pose_callback)

    def true_pose_callback(self, msg):
        """Updates the drone's true position and orientation."""
        self.body_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.body_orientation = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

        # Fix quaternion sign issue
        if self.body_orientation[3] < 0:
            self.body_orientation *= -1

    def semantic_callback(self, msg):
        """Processes the semantic image and identifies objects."""
        self.semantic_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        non_black_mask = np.any(self.semantic_image != [0, 0, 0], axis=-1)
        non_black_pixels = np.column_stack(np.where(non_black_mask))

        if non_black_pixels.size > 0:
            selected_pixels = non_black_pixels[np.random.choice(non_black_pixels.shape[0], min(500, non_black_pixels.shape[0]), replace=False)]
            if self.depth_image is not None:
                self.get_average_depth(selected_pixels)

    def depth_callback(self, msg):
        """Stores the latest depth image."""
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def get_average_depth(self, selected_pixels):
        """Retrieves the average depth value of 1000 randomly selected non-black pixels."""
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
            X = avg_depth  
            Y = -(np.mean(selected_pixels[:, 1]) - 160) * avg_depth / 120 
            Z = -(np.mean(selected_pixels[:, 0]) - 120) * avg_depth / 120
                                                                             
            object_body_frame = np.array([X, Y, Z])
            if self.body_position is not None and self.body_orientation is not None:
                object_world_frame = self.transform_to_world(object_body_frame)
                self.track_object(object_world_frame)

    def transform_to_world(self, object_body_frame):
        """Transforms the object's position from the body frame to the world frame."""
        R = quaternion_matrix(self.body_orientation)[:3, :3]
        object_world_frame = np.dot(R, object_body_frame) + self.body_position
        return object_world_frame

    def track_object(self, object_world_frame):
        """Tracks object positions and filters stable detections using unique object IDs."""
        target_coordinates = np.array([-59, 0.85, 6.6])  # Target coordinates
        distance_to_target = np.linalg.norm(object_world_frame - target_coordinates)
        # Publish the number of detected objects
        self.object_count_pub.publish(len(self.tracked_objects))
        # Neglect objects within 50 meters of the target coordinates
        if distance_to_target < 50:
            rospy.loginfo(f"Object too close to target coordinates, skipping: {object_world_frame}")
            return

        threshold = 30  # Distance threshold for associating detections with existing objects
        threshold2 = 2
        associated_object_id = None
        
        # Try to associate the new detection with an existing object.
        for obj_id, detections in self.tracked_objects.items():
            avg_position = np.mean(detections, axis=0)
            distance = np.linalg.norm(avg_position - object_world_frame)
            if distance < threshold:
                associated_object_id = obj_id
                break

        if associated_object_id is None:
            # No match found; create a new object ID.
            associated_object_id = self.next_object_id
            self.tracked_objects[associated_object_id] = []
            self.next_object_id += 1

        # Append the new detection.
        self.tracked_objects[associated_object_id].append(object_world_frame)

        # Keep last 10 detections for this object.
        if len(self.tracked_objects[associated_object_id]) > 20:
            self.tracked_objects[associated_object_id].pop(0)

        # Check stability: if 10 detections exist and they are within threshold, log and save the stable object.
        positions = np.array(self.tracked_objects[associated_object_id])
        if positions.shape[0] == 10:
            diffs = np.max(positions, axis=0) - np.min(positions, axis=0)
            if np.all(diffs <= threshold2):
                avg_position = np.mean(positions, axis=0)
                rospy.loginfo(f"Stable object {associated_object_id} detected at: {avg_position}")

                if associated_object_id not in self.logged_objects:
                    self.logged_objects.add(associated_object_id)
                    timestamp = rospy.get_time()
                    log_str = f"{timestamp}, Object {associated_object_id}, Position: {avg_position.tolist()}\n"
                    with open(self.output_file, "a") as file:
                        file.write(log_str)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    estimator = ObjectDetection()
    estimator.run()