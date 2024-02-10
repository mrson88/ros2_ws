from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(package='articubot_recognition', executable='yolov8_ros2_pt.py', output='screen'),
        # Node(package='articubot_recognition', executable='yolo_ros2_depth_test.py', output='screen'),
    ])