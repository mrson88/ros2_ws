#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer
from arduinobot_msgs.action import ArduinobotTask
# from moveit.planning import MoveItPy
# from moveit.core.robot_state import RobotState
# from moveit_python import MoveGroupInterface
# from moveit_msgs.msg import RobotState
# from moveit_py_interface import MoveItPy
from nav2_msgs.action import NavigateToPose
from arduinobot_msgs.action import MoveToPoint  # Update with your package name
from geometry_msgs.msg import Point
import sys
import time
from action_msgs.msg import GoalStatus

class TaskServer(Node):
    def __init__(self):
        super().__init__("task_server")
        self.get_logger().info("Starting the Server Python Task Server")
        # self.action_server = ActionServer(
        #     self, ArduinobotTask, "task_server", self.goalCallback
        # )
        # self._action_server = ActionServer(
        #     self,
        #     MoveToPoint,
        #     'task_server',
        #     self.goalCallback)
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'task_server',
            self.execute_callback)     
    def goalCallback(self, goal_handle):
        feedback_msg = MoveToPoint.Feedback()
        result = MoveToPoint.Result()

        # Dummy movement logic - replace with actual MoveIt 2 integration
        target_point = goal_handle.request.target_point
        self.get_logger().info(f'Moving to point: {target_point.x}, {target_point.y}, {target_point.z}')

        # Simulate some operation
        feedback_msg.distance_to_goal = 1.0  # Update with actual distance
        self.get_logger().info('Providing feedback...')
        goal_handle.publish_feedback(feedback_msg)

        # Assume success for simplicity
        result.success = True
        self.get_logger().info('Returning result...')
        return result
    def execute_callback(self, goal_handle):
        self.get_logger().info('Navigating to pose...')

        # Simulate navigation
        time.sleep(15)  # Simulate some operation that takes time
        
        goal_handle.succeed()

        result = NavigateToPose.Result()
        result.result = GoalStatus.STATUS_SUCCEEDED
        return result

def main(args=None):
    rclpy.init(args=args)
    task_server = TaskServer()
    rclpy.spin(task_server)


if __name__ == "__main__":
    main()
