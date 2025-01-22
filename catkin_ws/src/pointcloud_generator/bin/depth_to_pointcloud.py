#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge
import numpy as np

class DepthToPointCloud:
    def __init__(self):
        rospy.init_node('depth_to_pointcloud', anonymous=True)

        self.bridge = CvBridge()
        self.camera_info = None

        # Subscribers
        rospy.Subscriber('/realsense/depth/image', Image, self.depth_callback)
        rospy.Subscriber('/realsense/depth/camera_info', CameraInfo, self.camera_info_callback)

        # Publisher
        self.pointcloud_pub = rospy.Publisher('/camera/depth/points', PointCloud2, queue_size=1)

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, depth_msg):
        if self.camera_info is None:
            rospy.logwarn("Waiting for camera info...")
            return

        try:
            # Convert depth image to numpy array
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            height, width = depth_image.shape

            # Extract camera intrinsic parameters
            fx = self.camera_info.K[0]
            fy = self.camera_info.K[4]
            cx = self.camera_info.K[2]
            cy = self.camera_info.K[5]

            # Create a point cloud
            points = []
            for v in range(height):
                for u in range(width):
                    z = depth_image[v, u] / 1000.0  # Convert depth to meters
                    if z > 0:
                        x = (u - cx) * z / fx
                        y = (v - cy) * z / fy
                        points.append([x, y, z])

            # Convert to PointCloud2
            header = depth_msg.header
            pointcloud_msg = point_cloud2.create_cloud_xyz32(header, points)

            # Publish the point cloud
            self.pointcloud_pub.publish(pointcloud_msg)

        except Exception as e:
            rospy.logerr("Error converting depth to point cloud: %s", str(e))


if __name__ == '__main__':
    try:
        DepthToPointCloud()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
