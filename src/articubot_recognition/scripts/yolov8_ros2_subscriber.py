#!/usr/bin/env python3

import cv2
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from yolov8_msgs.msg import Yolov8Inference

import sys
import os
import time
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import statistics as st

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.subscription = self.create_subscription(
            Image,
            'color/image_raw',
            self.camera_callback,
            10)
        self.subscription 

    def camera_callback(self, data):
        global img
        img = bridge.imgmsg_to_cv2(data, "bgr8")

class Yolo_subscriber(Node):

    def __init__(self):
        super().__init__('yolo_subscriber')

        self.subscription = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.yolo_callback,
            10)
        self.subscription 

        self.cnt = 0

        self.img_pub = self.create_publisher(Image, "/inference_result_cv2", 1)
        self.W=640
        self.H=480
        self.config=rs.config()
        self.config.enable_stream(rs.stream.depth, self.W, self.H, rs.format.z16, 30)
        self.pipeline = rs.pipeline()
        self.profile = self.pipeline.start(self.config)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)



    def yolo_callback(self, data):
        global img
        time1 = time.time()
        self.frames = self.pipeline.wait_for_frames()
        self.aligned_frames = self.align.process(self.frames)
        self.depth_frame = self.aligned_frames.get_depth_frame()
        self.depth_image = np.asanyarray(self.depth_frame.get_data())
        self.depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.08), cv2.COLORMAP_JET)
        for r in data.yolov8_inference:
            
        
            class_name = r.class_name
            top = r.top
            left = r.left
            bottom = r.bottom
            right = r.right
            self.depth = self.depth_image[int((top+left)/2):int((top+left)/2+1), int((bottom+right)/2):int((bottom+right)/2+1)].astype(float)
            self.depth_crop = self.depth.copy()
            if self.depth_crop.size == 0:
                continue
            self.depth_res = self.depth_crop[self.depth_crop != 0]


            # Get data scale from the device and convert to meters
            self.depth_scale = self.profile.get_device().first_depth_sensor().get_depth_scale()
            self.depth_res = self.depth_res * self.depth_scale

            if self.depth_res.size == 0:
                continue
            # print(depth_res)
            self.dist = st.mean(self.depth_res)

            # yolo_subscriber.get_logger().info(f"{self.cnt} {class_name} : {top}, {left}, {bottom}, {right}")
            cv2.putText(img, text = f"{round(self.dist,2)}", org=(int(top), int(left)+30),
                        fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale = 0.7, color = (0, 0, 255),
                        thickness = 2, lineType=cv2.LINE_4)

            cv2.rectangle(img, (top, left), (bottom, right), (255, 255, 0))
            self.cnt += 1
        time2 = time.time()
        text_fps=f"FPS : {int(1/(time2-time1))}"
        cv2.putText(img, text = text_fps, org=(10,20),
            fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale = 0.7, color = (0, 0, 255),
            thickness = 2, lineType=cv2.LINE_4)
        self.cnt = 0
        img_msg = bridge.cv2_to_imgmsg(img)  
        self.img_pub.publish(img_msg)


if __name__ == '__main__':
    rclpy.init(args=None)
    yolo_subscriber = Yolo_subscriber()
    camera_subscriber = Camera_subscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(yolo_subscriber)
    executor.add_node(camera_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    rate = yolo_subscriber.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()
