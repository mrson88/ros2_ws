#!/usr/bin/env python3
import cv2
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from arduinobot_msgs.msg import InferenceResult
from arduinobot_msgs.msg import Yolov8Inference
import sys
import os
import time
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import statistics as st
from geometry_msgs.msg import Twist
bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.model = YOLO('~/ros2_ws/src/yolobot_recognition/scripts/yolov8n.pt')

        self.yolov8_inference = Yolov8Inference()

        self.subscription = self.create_subscription(
            Image,
            'color/image_raw',
            self.camera_callback,
            10)
        self.subscription 
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # self.subscription_depth = self.create_subscription(
        #     Point,
        #     'depth/image_rect_raw',
        #     self.depth_camera_callback,
        #     10)
        # self.subscription_depth
        # self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

    def camera_callback(self, data):
        msg = Twist()

        img = bridge.imgmsg_to_cv2(data, "bgr8")
        # depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        time1 = time.time()
        results = self.model(img)

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = camera_subscriber.get_clock().now().to_msg()

        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inference_result = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                c = box.cls
                self.inference_result.class_name = self.model.names[int(c)]
                if self.inference_result.class_name=="person":
                    msg.angular.z=0.2
                msg.linear.x=0.1

                

                self.inference_result.top = int(b[0])
                self.inference_result.left = int(b[1])
                self.inference_result.bottom = int(b[2])
                self.inference_result.right = int(b[3])
                self.pixel_x = int((self.inference_result.top+self.inference_result.right)/2)
                self.pixel_y = int((self.inference_result.bottom+self.inference_result.left)/2)
                self.yolov8_inference.yolov8_inference.append(self.inference_result)

                # self.depth_value = depth_image[self.pixel_y, self.pixel_x]
                # cv2.putText(img, text = f"{round(self.depth_value,2)}", org=(int(self.inference_result.top), int(self.inference_result.left)+30),
                #         fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale = 0.7, color = (0, 0, 255),
                #         thickness = 2, lineType=cv2.LINE_4)

            #camera_subscriber.get_logger().info(f"{self.yolov8_inference}")

        annotated_frame = results[0].plot()
        
        
        time2 = time.time()
        text_fps=f"FPS : {int(1/(time2-time1))}"
        cv2.putText(annotated_frame, text = text_fps, org=(10,20),
            fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale = 0.7, color = (0, 0, 255),
            thickness = 2, lineType=cv2.LINE_4)
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)  
        self.img_pub.publish(img_msg)
        # self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()


    # def depth_camera_callback(self,data : Point):
    #     self.depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    #     self.depth_value = self.depth_image[self.pixel_y, self.pixel_x]
    #     camera_subscriber.get_logger().info(f"{self.depth_value}")
    #     return self.depth_value
        
if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()
