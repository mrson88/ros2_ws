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
import statistics as st
from geometry_msgs.msg import Twist
import mediapipe as mp
from cvzone.HandTrackingModule import HandDetector
from cvzone.ClassificationModule import Classifier
import math
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from arduinobot_msgs.action import ArduinobotTask
from utils.cvfpscalc import CvFpsCalc
from model.keypoint_classifier import KeyPointClassifier
from model.point_history_classifier import PointHistoryClassifier
import csv
import copy
import itertools
from collections import Counter
from collections import deque
from app import *
# import tensorflow as tf
# bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        self.pixel_x = 0
        self.pixel_y = 0
        self.depth_image=[]
        self.depth_value = 0
        self.model = YOLO('~/ros2_ws/src/yolobot_recognition/scripts/yolov8m.pt')

        self.yolov8_inference = Yolov8Inference()

        self.subscription = self.create_subscription(
            Image,
            'color/image_raw',
            self.camera_callback,
            10)
        self.subscription 
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.subscription_depth = self.create_subscription(
            Image,
            'depth/image_rect_raw',
            self.depth_camera_callback,
            10)
        self.subscription_depth
        # self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)
        self.detector = HandDetector(maxHands=1)
        self.offset = 20
        self.imgSize = 300
        self.counter = 0
        self.classifier = Classifier("/home/mrson/ros2_ws/src/articubot_recognition/scripts/Model/keras_model.h5" , "/home/mrson/ros2_ws/src/articubot_recognition/scripts/Model/labels.txt")
        self.labels = ["Hello","I love you","No","Okay","Please","Thank you","Yes"]
        self._action_client = ActionClient(self, ArduinobotTask, 'task_server')
        self.weight=848
        self.hight=480
        self.keypoint_classifier = KeyPointClassifier()
        self.point_history_classifier = PointHistoryClassifier()    
        with open('/home/mrson/ros2_ws/src/articubot_recognition/scripts/model/keypoint_classifier/keypoint_classifier_label.csv',
                    encoding='utf-8-sig') as f:
                self.keypoint_classifier_labels = csv.reader(f)
                self.keypoint_classifier_labels = [
                    row[0] for row in self.keypoint_classifier_labels
                ]
        with open(
                    '/home/mrson/ros2_ws/src/articubot_recognition/scripts/model/point_history_classifier/point_history_classifier_label.csv',
                    encoding='utf-8-sig') as f:
                self.point_history_classifier_labels = csv.reader(f)
                self.point_history_classifier_labels = [
                    row[0] for row in self.point_history_classifier_labels
                ] 
        self.history_length = 16
        self.point_history = deque(maxlen=self.history_length)
        self.finger_gesture_history = deque(maxlen=self.history_length)
        self.class_id=0
        self.confidence=0
        self.xcentre=0
        self.input_mean = 127.5
        self.input_std = 127.5
        self.result=False
        self.detect=True 
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
        max_num_hands=1,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.5,
    )
    def detect_hand(self,frame):
            use_brect = True

            image = cv2.flip(frame, 1)  # Mirror display
            debug_image = copy.deepcopy(image)
            # Detection implementation #############################################################
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image.flags.writeable = False
            results = self.hands.process(image)
            image.flags.writeable = True
            

            #  ####################################################################
            if results.multi_hand_landmarks is not None:
                for hand_landmarks, handedness in zip(results.multi_hand_landmarks,
                                                    results.multi_handedness):
                    # Bounding box calculation
                    brect = calc_bounding_rect(debug_image, hand_landmarks)
                    # Landmark calculation
                    landmark_list = calc_landmark_list(debug_image, hand_landmarks)

                    # Conversion to relative coordinates / normalized coordinates
                    pre_processed_landmark_list = pre_process_landmark(
                        landmark_list)
                    pre_processed_point_history_list = pre_process_point_history(
                        debug_image, self.point_history)
                    # Hand sign classification
                    hand_sign_id = self.keypoint_classifier(pre_processed_landmark_list)
                    if hand_sign_id == 2:  # Point gesture
                        self.point_history.append(landmark_list[8])
                    else:
                        self.point_history.append([0, 0])

                    # Finger gesture classification
                    finger_gesture_id = 0
                    point_history_len = len(pre_processed_point_history_list)
                    if point_history_len == (self.history_length * 2):
                        finger_gesture_id = self.point_history_classifier(
                            pre_processed_point_history_list)

                    # Calculates the gesture IDs in the latest detection
                    self.finger_gesture_history.append(finger_gesture_id)
                    most_common_fg_id = Counter(
                        self.finger_gesture_history).most_common()

                    # Drawing part
                    debug_image = draw_bounding_rect(use_brect, debug_image, brect)
                    debug_image = draw_landmarks(debug_image, landmark_list)
                    debug_image = draw_info_text(
                        debug_image,
                        brect,
                        handedness,
                        self.keypoint_classifier_labels[hand_sign_id],
                        self.point_history_classifier_labels[most_common_fg_id[0][0]],
                    )
            else:
                self.point_history.append([0, 0])
    
            return debug_image

    def detect_hand_one(self,img):
        index=0
        imgCrop=[]
        img=cv2.resize(img,(self.weight,self.hight))
        imgOutput = img.copy()
        hands, img = self.detector.findHands(img)

        if hands:
            hand = hands[0]

            x, y, w, h = hand['bbox']
            imgWhite = np.ones((self.imgSize, self.imgSize, 3), np.uint8)*255
            if y-self.offset>0 and y + h + self.offset<self.hight and x-self.offset>0 and x + w + self.offset<self.weight:
                imgCrop = img[y-self.offset:y + h + self.offset, x-self.offset:x + w + self.offset]
            aspectRatio = h / w
            if len(imgCrop)>0:
                if aspectRatio > 1:
                    k = self.imgSize / h
                    wCal = math.ceil(k * w)
                    imgResize = cv2.resize(imgCrop, (wCal, self.imgSize))
                    imgResizeShape = imgResize.shape
                    wGap = math.ceil((self.imgSize-wCal)/2)
                    imgWhite[:, wGap: wCal + wGap] = imgResize
                    prediction , index = self.classifier.getPrediction(imgWhite, draw= False)
                    print(prediction, index)

                else:
                    k = self.imgSize / w
                    hCal = math.ceil(k * h)
                    imgResize = cv2.resize(imgCrop, (self.imgSize, hCal))
                    imgResizeShape = imgResize.shape
                    hGap = math.ceil((self.imgSize - hCal) / 2)
                    imgWhite[hGap: hCal + hGap, :] = imgResize
                    prediction , index = self.classifier.getPrediction(imgWhite, draw= False)
            cv2.rectangle(imgOutput,(x-self.offset,y-self.offset-70),(x -self.offset+400, y - self.offset+60-50),(0,255,0),cv2.FILLED)
            cv2.putText(imgOutput,self.labels[index],(x,y-30),cv2.FONT_HERSHEY_COMPLEX,2,(0,0,0),2) 
            cv2.rectangle(imgOutput,(x-self.offset,y-self.offset),(x + w + self.offset, y+h + self.offset),(0,255,0),4)  
            # self._action_client.wait_for_server()
            # future=goal_handle_result=self.send_goal(1)
            # rclpy.spin_until_future_complete(self._action_client, future)
            if self.detect:
                if self.labels[index]=="Thank you":
                    # self._action_client.wait_for_server()
                    self.send_goal(2)     
                elif self.labels[index]=="Hello":
                    self.send_goal(1)
                    time.sleep=1
                elif self.labels[index]=="No":
                    self.send_goal(0)  
                    time.sleep=1
        return imgOutput
    
    def send_goal(self, order):
        self.detect=False
        goal_msg = ArduinobotTask.Goal()
        goal_msg.task_number = order
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.get_logger().info('Result: {0}'.format(self._send_goal_future.add_done_callback(self.goal_response_callback)))
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        time.sleep=1

    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.success))
        if result.success:
            self.detect=True


    def feedback_callback(self, feedback_msg):
            feedback = feedback_msg.feedback
            self.get_logger().info('Received feedback:')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)      

    def yolov8_reg(self,img):
        results = self.model(img, conf=0.5)
        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = camera_subscriber.get_clock().now().to_msg()
        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inference_result = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                c = box.cls
                self.inference_result.class_name = self.model.names[int(c)]
                self.inference_result.top = int(b[0])
                self.inference_result.left = int(b[1])
                self.inference_result.bottom = int(b[2])
                self.inference_result.right = int(b[3])
                self.pixel_x = int((self.inference_result.top+self.inference_result.bottom)/2)
                self.pixel_y = int((self.inference_result.right+self.inference_result.left)/2)
                self.yolov8_inference.yolov8_inference.append(self.inference_result)
                if len(self.depth_image)!=0:
                    self.depth_value = self.depth_image[self.pixel_y, self.pixel_x]
                    # if self.inference_result.class_name=="person":

                    #     node=rclpy.create_node('movebase_client')
                    #     client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
                    #     goal_msg = NavigateToPose.Goal()
                    #     goal_msg.pose.header.frame_id = 'base_link'  # Set the frame according to your robot's frame
                    #     goal_msg.pose.pose.position.x = 1.0
                    #     goal_msg.pose.pose.orientation.w = 1.0 
                    #     self.get_logger().info(f"move robot")
                    #     goal_handle_future = client.send_goal_async(goal_msg)
                        # msg.angular.z=0.2
                        # msg.linear.x=0.1
                    #     self.get_logger().info(f"class_name %= {self.inference_result.class_name}")
                    #     # while rclpy.ok():
                    #     rclpy.spin_once(node)
                    #     if goal_handle_future.done():
                    #         goal_handle = goal_handle_future.result()
                    #         if goal_handle.accepted:
                    #             print("Goal accepted.")
                    #             result_future = goal_handle.get_result_async()
                    #             rclpy.spin_until_future_complete(node, result_future)
                    #             result = result_future.result().result
                    #             if result != False:
                    #                 print("Goal execution done!")
                    #                 self.get_logger().info("Goal execution done!")
                    #             else:
                    #                 print("Goal execution failed!")
                    #                 self.get_logger().info("Goal execution failed!")
                                
                    #         else:
                    #             self.get_logger().info("Goal rejected.")
                    #             print("Goal rejected.")
                                
                    #         # node.destroy_node()
                cv2.putText(img, text = f"kc: {round(self.depth_value,3)}", org=(int(self.inference_result.top), int(self.inference_result.left)+30),
                        fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale = 0.5, color = (0, 255, 0),
                        thickness = 1, lineType=cv2.LINE_4)
        image = results[0].plot()

        return image
    def camera_callback(self, data):
        msg = Twist()
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        time1 = time.time()
        self.publisher_.publish(msg)
        annotated_frame=self.yolov8_reg(img)
        # annotated_frame=self.detect_hand_one(annotated_frame)
        # annotated_frame=self.detect_hand(annotated_frame)
        # annotated_frame,self.class_id,self.confidence,self.xcentre=start(img)
        
        
        time2 = time.time()
        text_fps=f"FPS : {int(1/(time2-time1))}"
        cv2.putText(annotated_frame, text = text_fps, org=(10,20),
            fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale = 0.5, color = (0, 0, 255),
            thickness = 1, lineType=cv2.LINE_4)
        img_msg = self.bridge.cv2_to_imgmsg(annotated_frame)  
        self.img_pub.publish(img_msg)
        # self.yolov8_pub.publish(self.yolov8_inference)
        # self.yolov8_inference.yolov8_inference.clear()


    def depth_camera_callback(self,data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        # self.depth_value = self.depth_image[self.pixel_y, self.pixel_x]
        # self.get_logger().info(f"Depth value at depth {self.depth_value}")
        
if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()
