#!/usr/bin/env python
import cv2
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
        self.pixel_x = 100
        self.pixel_y = 150
        super().__init__('camera_depth')
        self.get_logger().info("Depth value at start ")
        # self.publisher_ = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.bridge = CvBridge()
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)
        # self.sub = self.create_subscription('/camera/depth/image_raw', Image, self.depth_callback,10)
        self.sub =self.create_subscription(
            Image,
            'depth/image_rect_raw',
            self.depth_callback,
            10)
        self.sub
        # self.pub = self.create_publisher('/distance', float, 1)
        self.pub_ = self.create_publisher(Float64, "/distance", 10)
        self.get_logger().info("Depth value at start-end ")

    def depth_callback(self, data):
        self.get_logger().info("Depth value at depth ")
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
        self.get_logger().info("Depth value at caculate ")
        udist = self.depth_frame[self.y1, self.x1]
        vdist = self.depth_frame[self.y2, self.x2]

        point1 = rs.rs2_deproject_pixel_to_point(self.color_intrin, [self.x1, self.y1], udist)
        point2 = rs.rs2_deproject_pixel_to_point(self.color_intrin, [self.x2, self.y2], vdist)

        dist = math.sqrt(math.pow(point1[0] - point2[0], 2) + math.pow(point1[1] - point2[1], 2) + math.pow(point1[2] - point2[2], 2))
        self.get_logger().info("Depth value at row %d, col %d: %u", point1, point2, dist)

        return dist

if __name__ == '__main__':
    rclpy.init(args=None)
    distance_measure = Distance_measure()
    rclpy.spin(distance_measure)
    rclpy.shutdown()



# # #!/usr/bin/env python
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import numpy as np
# import pyrealsense2 as rs


# class RealSenseDepthNode(Node):
#     def __init__(self):
#         super().__init__('realsense_depth_node')
#         self.publisher_ = self.create_publisher(Image, 'depth_image', 10)
#         self.subscription_ = self.create_subscription(
#             Image, '/camera/depth/image_raw', self.depth_image_callback, 10)
#         self.bridge = CvBridge()
#         self.pipeline = rs.pipeline()
#         config = rs.config()
#         config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#         self.pipeline.start(config)

#     def publish_depth_image(self):
#         frames = self.pipeline.wait_for_frames()
#         depth_frame = frames.get_depth_frame()
#         depth_image = np.asanyarray(depth_frame.get_data())
#         msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")
#         msg.header.stamp = self.get_clock().now().to_msg()
#         self.publisher_.publish(msg)

#     def depth_image_callback(self, msg):
#         depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
#         row = 100  # Example row
#         col = 200  # Example column
#         depth_value = depth_image[row, col]
#         self.get_logger().info("Depth value at row %d, col %d: %u", row, col, depth_value)


# def main(args=None):
#     rclpy.init(args=args)
#     node = RealSenseDepthNode()
#     try:
#         while rclpy.ok():
#             rclpy.spin_once(node)
#             node.publish_depth_image()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
