#!/usr/bin/env python
import cv2
from pandas import Float32Dtype
# import rospy
import cv_bridge
import numpy as np
import pyrealsense2 as rs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from example_interfaces.msg import Float64
class Distance_measure(Node):
    def __init__(self):
        super().__init__('camera_depth')
        self.pipeline = rs.pipeline()
        self.pipeline.start(rs.config())
        self.W=640
        self.H=480
        # self.config=rs.config()
        # self.config.enable_stream(rs.stream.depth, self.W, self.H, rs.format.z16, 30)
        self.pipeline = rs.pipeline()
        self.profile = self.pipeline.start(self.config)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        self.bridge = CvBridge()
        # self.sub = self.create_subscription('/camera/depth/image_raw', Image, self.depth_callback,10)
        self.sub =self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10)
        self.sub
        # self.pub = self.create_publisher('/distance', float, 1)
        self.pub_ = self.create_publisher(Float64, "/distance", 10)

    def depth_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        self.depth_frame = cv2.convertScaleAbs(cv_image, alpha=0.01)

        self.color_intrin = rs.video_stream_profile(self.pipeline.get_active_profile().get_stream(rs.stream.color)).get_intrinsics()

        self.x1 = 480
        self.y1 = 550
        self.x2 = 810
        self.y2 = self.y1
        ans = self.calculate_distance()
        print(ans)
        self.pub.publish(ans)

    def calculate_distance(self):
        udist = self.depth_frame[self.y1, self.x1]
        vdist = self.depth_frame[self.y2, self.x2]

        point1 = rs.rs2_deproject_pixel_to_point(self.color_intrin, [self.x1, self.y1], udist)
        point2 = rs.rs2_deproject_pixel_to_point(self.color_intrin, [self.x2, self.y2], vdist)

        dist = math.sqrt(math.pow(point1[0] - point2[0], 2) + math.pow(point1[1] - point2[1], 2) + math.pow(point1[2] - point2[2], 2))
        return dist

if __name__ == '__main__':
    rclpy.init(args=None)
    distance_measure = Distance_measure()
    rclpy.spin(distance_measure)
    rclpy.shutdown()