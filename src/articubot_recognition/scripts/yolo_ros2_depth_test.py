#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from arduinobot_msgs.msg import InferenceResult
import sys
import os
import time
import numpy as np
import pyrealsense2 as rs
import statistics as st
from geometry_msgs.msg import Twist
import math
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from arduinobot_msgs.action import ArduinobotTask
from utils.cvfpscalc import CvFpsCalc
from ament_index_python.packages import get_package_share_directory
# from tflite_runtime.interpreter import Interpreter
import tensorflow as tf
from collections import Counter
from collections import deque
from pathlib import Path
from arduinobot_msgs.action import MoveToPoint  # Update with your package name
from geometry_msgs.msg import Point

import mediapipe as mp
from cvzone.HandTrackingModule import HandDetector
from cvzone.ClassificationModule import Classifier
from utils.cvfpscalc import CvFpsCalc
from model.keypoint_classifier import KeyPointClassifier
from model.point_history_classifier import PointHistoryClassifier
import csv
import copy
import itertools

from app import *
from Facerec_QT.face_rec import FaceRecognition,read_db,write_db

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge=CvBridge()
        self.pixel_x = 0
        self.pixel_y = 0
        self.depth_image=[]
        self.depth_value = 0
        self.package_share_dir = get_package_share_directory("articubot_recognition")
        self.model_file = os.path.join(self.package_share_dir, "data","lite-model_ssd_mobilenet_v1_100_320_uint8_nms_1.tflite")
        self.label_file = os.path.join(self.package_share_dir, "data","labels.txt")
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
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)
        self.offset = 20
        self.imgSize = 300
        self.counter = 0
        self.detector = HandDetector(maxHands=1)
        self._action_client = ActionClient(self, ArduinobotTask, 'task_server')
        self.weight=640
        self.hight=480
        self.imH=320
        self.imW=320
        self.frame_height = 480
        self.frame_width = 640
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
        max_num_hands=2,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.5,

    )
        # self.model_file_hand = os.path.join(self.package_share_dir, "scripts","Model/keras_model.h5")
        # self.label_file_hand = os.path.join(self.package_share_dir, "scripts","Model/labels.txt")
        # self.classifier = Classifier(self.model_file_hand  , self.label_file_hand)
        # self.labels = ["Hello","I love you","No","Okay","Please","Thank you","Yes"]
        self.recognition_on=True
        self.face_recognition = FaceRecognition(0.7, self.frame_height, self.frame_width)
        self.registration_data = None
        self.detect=True
        self.keypoint_classifier = KeyPointClassifier()
        self.point_history_classifier = PointHistoryClassifier() 
        self.key_point_label_dir=os.path.join(self.package_share_dir, "scripts","model/keypoint_classifier/keypoint_classifier_label.csv")
        self.point_history_label_dir=os.path.join(self.package_share_dir, "scripts","model/point_history_classifier/point_history_classifier_label.csv")
        with open(self.key_point_label_dir,
                    encoding='utf-8-sig') as f:
                self.keypoint_classifier_labels = csv.reader(f)
                self.keypoint_classifier_labels = [
                    row[0] for row in self.keypoint_classifier_labels
                ]
        with open(self.point_history_label_dir,
                    encoding='utf-8-sig') as f:
                self.point_history_classifier_labels = csv.reader(f)
                self.point_history_classifier_labels = [
                    row[0] for row in self.point_history_classifier_labels
                ] 
        # self.label_file = os.path.join(self.package_share_dir, "data","labels.txt")
        self._action_client_move = ActionClient(self, MoveToPoint, 'task_server')
        self.history_length = 16
        self.point_history = deque(maxlen=self.history_length)
        self.finger_gesture_history = deque(maxlen=self.history_length)
        self.prev_status=""
    def send_goal_move(self, x, y, z):
        goal_msg = MoveToPoint.Goal()
        goal_msg.target_point.x = x
        goal_msg.target_point.y = y
        goal_msg.target_point.z = z

        self._action_client_move.wait_for_server()
        self._send_goal_future = self._action_client_move.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}')
        if result.success:
            self.detect=True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance to goal: {feedback.distance_to_goal}')

    def detect_tflite(self, data):
        
        # frame = self.bridge.imgmsg_to_cv2(data,'bgr8')
        
        frame = cv2.cvtColor(data, cv2.COLOR_BGR2RGB)
        frame = cv2.resize(frame,(self.imH,self.imW))
        # frame = cv2.resize(frame,(320,320))
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        input_image = frame
        # input_image = cv2.resize(input_image,(self.imH,self.imW))
        input_image = input_image.reshape(1 , input_image.shape[0],input_image.shape[1],input_image.shape[2])
        input_image = input_image.astype(np.uint8)


        interpreter = tf.lite.Interpreter(model_path=self.model_file)
        interpreter.allocate_tensors()

        # Get input and output tensors.
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()

        input_data = np.array(input_image, dtype=np.uint8)
        interpreter.set_tensor(input_details[0]['index'], input_data)

        text_file = open(self.label_file,"r")
        label_array = text_file.readlines()

        interpreter.invoke()
        boxes = interpreter.get_tensor(output_details[0]['index'])[0]
        predicted_labels = interpreter.get_tensor(output_details[1]['index'])[0]
        predicted_scores = interpreter.get_tensor(output_details[2]['index'])[0]
        top_score = predicted_scores[0]
        top_label = label_array[int(predicted_labels[0]) ]

        for i in range(len(predicted_scores)):
            if ((predicted_scores[i] > 0.6) and (predicted_scores[i] <= 1.0)):
                # Get bounding box coordinates and draw box
                # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
                ymin = int(max(1, (boxes[i][0] * self.imH)))
                xmin = int(max(1, (boxes[i][1] * self.imW)))
                ymax = int(min(self.imH, (boxes[i][2] * self.imH)))
                xmax = int(min(self.imW, (boxes[i][3] * self.imW)))
                self.pixel_x = int((xmin + xmax) / 2)
                self.pixel_y = int((ymin + ymax) / 2)
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (10, 255, 0), 1)


                # Draw label
                object_name = label_array[int(predicted_labels[i])]
                # self.get_logger().info(object_name)
                confidence=int(predicted_scores[i] * 100)# Look up object name from "labels" array using class index
                label = '%s: %d%%' % (object_name, confidence)  # Example: 'person: 72%'
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)  # Get font size
                label_ymin = max(ymin, labelSize[1] + 10)  # Make sure not to draw label too close to top of window
                cv2.rectangle(frame, (xmin, label_ymin - labelSize[1] - 10),
                              (xmin + labelSize[0], label_ymin + baseLine - 10), (255, 255, 255),
                              cv2.FILLED)  # Draw white box to put label text in
                cv2.putText(frame, label, (xmin, label_ymin - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0),
                            2)  # Draw label text
                
                if len(self.depth_image)!=0:
                    self.depth_value = self.depth_image[self.pixel_y, self.pixel_x]
                    if 'person' in str(object_name) and self.depth_value>2000:
                        

                        # node=rclpy.create_node('movebase_client')
                        # client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
                        # goal_msg = NavigateToPose.Goal()
                        # goal_msg.pose.header.frame_id = 'base_link'  # Set the frame according to your robot's frame
                        # goal_msg.pose.pose.position.x = 1.0
                        # goal_msg.pose.pose.orientation.w = 1.0 
                        # self.get_logger().info(f"move robot")
                        # goal_handle_future = client.send_goal_async(goal_msg)
                        msg = Twist()
                        # msg.angular.z=0.02
                        # msg.linear.x=0.05
                        self.publisher_.publish(msg)
                        self.get_logger().info(f"class_name %= {object_name}")
                        self.get_logger().info(f"depth %= {self.depth_value}")
                        # self.send_goal_move(1.0, 2.0, 3.0)
                        # # while rclpy.ok():
                        # rclpy.spin_once(node)
                        # if goal_handle_future.done():
                        #     goal_handle = goal_handle_future.result()
                        #     if goal_handle.accepted:
                        #         print("Goal accepted.")
                        #         result_future = goal_handle.get_result_async()
                        #         rclpy.spin_until_future_complete(node, result_future)
                        #         result = result_future.result().result
                        #         if result != False:
                        #             print("Goal execution done!")
                        #             self.get_logger().info("Goal execution done!")
                        #         else:
                        #             print("Goal execution failed!")
                        #             self.get_logger().info("Goal execution failed!")
                                
                        #     else:
                        #         self.get_logger().info("Goal rejected.")
                        #         print("Goal rejected.")
                                
                            # node.destroy_node()
            else:
                object_name=''
                xcentre=0
                confidence=0
        return frame
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
                    self.get_logger().info(str(self.keypoint_classifier_labels[hand_sign_id]))
                if self.detect and self.keypoint_classifier_labels[hand_sign_id]!=self.prev_status:
                    if self.keypoint_classifier_labels[hand_sign_id]=="Open" :
                        self.prev_status="Open"
                        self.send_goal(0)  
                    elif self.keypoint_classifier_labels[hand_sign_id]=="Close":
                        self.prev_status="Close"
                        self.send_goal(2)
                    elif self.keypoint_classifier_labels[hand_sign_id]=="Pointer":
                        self.prev_status="Pointer"
                        self.send_goal(1)
            else:
                self.point_history.append([0, 0])
    
            return debug_image   

    # def detect_hand_one(self,img):
    #     index=0
    #     imgCrop=[]
    #     img=cv2.resize(img,(self.weight,self.hight))
    #     imgOutput = img.copy()
    #     hands, img = self.detector.findHands(img)

    #     if hands:
    #         hand = hands[0]

    #         x, y, w, h = hand['bbox']
    #         imgWhite = np.ones((self.imgSize, self.imgSize, 3), np.uint8)*255
    #         if y-self.offset>0 and y + h + self.offset<self.hight and x-self.offset>0 and x + w + self.offset<self.weight:
    #             imgCrop = img[y-self.offset:y + h + self.offset, x-self.offset:x + w + self.offset]
    #         aspectRatio = h / w
    #         if len(imgCrop)>0:
    #             if aspectRatio > 1:
    #                 k = self.imgSize / h
    #                 wCal = math.ceil(k * w)
    #                 imgResize = cv2.resize(imgCrop, (wCal, self.imgSize))
    #                 imgResizeShape = imgResize.shape
    #                 wGap = math.ceil((self.imgSize-wCal)/2)
    #                 imgWhite[:, wGap: wCal + wGap] = imgResize
    #                 prediction , index = self.classifier.getPrediction(imgWhite, draw= False)
    #                 print(prediction, index)

    #             else:
    #                 k = self.imgSize / w
    #                 hCal = math.ceil(k * h)
    #                 imgResize = cv2.resize(imgCrop, (self.imgSize, hCal))
    #                 imgResizeShape = imgResize.shape
    #                 hGap = math.ceil((self.imgSize - hCal) / 2)
    #                 imgWhite[hGap: hCal + hGap, :] = imgResize
    #                 prediction , index = self.classifier.getPrediction(imgWhite, draw= False)
    #         cv2.rectangle(imgOutput,(x-self.offset,y-self.offset-70),(x -self.offset+400, y - self.offset+60-50),(0,255,0),cv2.FILLED)
    #         cv2.putText(imgOutput,self.labels[index],(x,y-30),cv2.FONT_HERSHEY_COMPLEX,2,(0,0,0),2) 
    #         cv2.rectangle(imgOutput,(x-self.offset,y-self.offset),(x + w + self.offset, y+h + self.offset),(0,255,0),4)  
    #         # self._action_client.wait_for_server()
    #         # future=goal_handle_result=self.send_goal(1)
    #         # rclpy.spin_until_future_complete(self._action_client, future)
    #         if self.detect:
    #             if self.labels[index]=="Thank you":
    #                 # self._action_client.wait_for_server()
    #                 self.send_goal(1)  
    #                 # time.sleep=1
    #                 # self.detect=True 
    #             elif self.labels[index]=="Hello":
    #                 self.send_goal(2)
    #                 # time.sleep=1
    #                 # self.detect=True
    #             elif self.labels[index]=="No":
    #                 self.send_goal(0)  
    #                 # time.sleep=1
    #                 # self.detect=True
                
    #     return imgOutput

    def send_goal(self, order):
        self.detect=False
        goal_msg = ArduinobotTask.Goal()
        goal_msg.task_number = order
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def camera_callback(self, data):
        # msg = Twist()
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        time1 = time.time()
        # self.publisher_.publish(msg)
        # img=self.detect_hand_one(img)
        img=self.detect_hand(img)
        annotated_frame=self.detect_tflite(img)
        
        # annotated_frame=self.detect_hand(img)
        # annotated_frame,self.class_id,self.confidence,self.xcentre=start(img)
        
        
        time2 = time.time()
        text_fps=f"FPS : {int(1/(time2-time1))}"
        cv2.putText(annotated_frame, text = text_fps, org=(10,20),
            fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale = 0.5, color = (0, 0, 255),
            thickness = 1, lineType=cv2.LINE_4)
        processed_frame, ids = self.face_recognition.process_frame(annotated_frame, self.recognition_on, self.registration_data)
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)  
        self.img_pub.publish(img_msg)
        # self.yolov8_pub.publish(self.yolov8_inference)
        # self.yolov8_inference.yolov8_inference.clear()


    def depth_camera_callback(self,data):
        self.depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        # self.depth_value = self.depth_image[self.pixel_y, self.pixel_x]
        # self.get_logger().info(f"Depth value at depth {self.depth_value}")
        
if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()
