import rclpy
from rclpy.node import Node
from moveit_python import MoveGroupInterface

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('moveit_example')

    # Initialize MoveGroupInterface
    move_group = MoveGroupInterface("robot_arm", "base_link", node=node)

    # Plan and execute a motion
    move_group.moveToJointPosition(['joint1', 'joint2', 'joint3'], [1.0, 2.0, 3.0])

    rclpy.shutdown()

if __name__ == '__main__':
    main()
