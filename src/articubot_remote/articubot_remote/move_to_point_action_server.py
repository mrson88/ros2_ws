import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from arduinobot_msgs.action import MoveToPoint  # Update with your package name
from geometry_msgs.msg import Point
import sys

class MoveToPointActionServer(Node):

    def __init__(self):
        super().__init__('move_to_point_action_server')
        self._action_server = ActionServer(
            self,
            MoveToPoint,
            'move_to_point',
            self.execute_callback)

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = MoveToPoint.Feedback()
        result = MoveToPoint.Result()

        # Dummy movement logic - replace with actual MoveIt 2 integration
        target_point = goal_handle.request.target_point
        self.get_logger().info(f'Moving to point: {target_point.x}, {target_point.y}, {target_point.z}')

        # Simulate some operation
        feedback_msg.distance_to_goal = 0.0  # Update with actual distance
        self.get_logger().info('Providing feedback...')
        await goal_handle.publish_feedback(feedback_msg)

        # Assume success for simplicity
        result.success = True
        self.get_logger().info('Returning result...')
        return result

def main(args=None):
    rclpy.init(args=args)
    move_to_point_action_server = MoveToPointActionServer()

    try:
        rclpy.spin(move_to_point_action_server)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
