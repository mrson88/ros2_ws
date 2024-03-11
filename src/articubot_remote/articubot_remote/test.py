import rclpy
from rclpy.node import Node
from moveit2 import MoveGroupInterface
from geometry_msgs.msg import PoseStamped

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')
        # Initialize MoveGroupInterface for the specified arm group.
        self.arm_move_group = MoveGroupInterface("arm_robot", "robot_description")

    def move_to_pose(self, pose_stamped):
        # Set the pose target for the arm.
        self.arm_move_group.set_pose_target(pose_stamped)

        # Plan and execute the motion.
        plan_success, motion_plan, planning_time, error_code = self.arm_move_group.plan_kinematic_path()
        if plan_success:
            self.arm_move_group.execute(motion_plan)
            self.get_logger().info("Movement executed successfully.")
        else:
            self.get_logger().info(f"Failed to execute movement. Error code: {error_code}")

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    
    try:
        # Define the pick pose.
        pick_pose = PoseStamped()
        pick_pose.header.frame_id = "world"  # Reference frame
        pick_pose.pose.position.x = 0.4  # Example positions
        pick_pose.pose.position.y = 0.1
        pick_pose.pose.position.z = 0.5
        pick_pose.pose.orientation.w = 1.0  # Assuming an upright orientation
        
        # Move to the pick pose.
        node.move_to_pose(pick_pose)
        
        # Define the place pose similarly and move to it.
        # place_pose = ...
        # node.move_to_pose(place_pose)
        
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
