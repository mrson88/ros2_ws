import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory

class GripperActionServer(Node):

    def __init__(self):
        super().__init__('gripper_action_server')
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'gripper_control',
            self.execute_callback
        )
        self.get_logger().info('Gripper Action Server has been started')

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received gripper trajectory goal')
        goal = goal_handle.request
        trajectory = goal.trajectory

        # Assuming one joint in the gripper
        if len(trajectory.joint_names) != 1:
            self.get_logger().error('Unexpected number of joints in trajectory')
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        # Assuming a simple gripper with only one joint
        if len(trajectory.points) != 1:
            self.get_logger().error('Unexpected number of trajectory points')
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        # Execute the trajectory
        point = trajectory.points[0]
        goal_position = point.positions[0]
        self.get_logger().info(f'Executing gripper trajectory to position: {goal_position}')

        # You would typically send commands to your gripper hardware here
        # Replace this with your actual code

        # Indicate that the action has been completed successfully
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        goal_handle.succeed(result)
        return result

def main(args=None):
    rclpy.init(args=args)
    gripper_action_server = GripperActionServer()
    rclpy.spin(gripper_action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
