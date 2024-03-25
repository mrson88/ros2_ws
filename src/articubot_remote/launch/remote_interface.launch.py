from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # task_server_node = Node(
    #     package="articubot_remote",
    #     executable="task_server_node",
    # )
    # task_server_node = Node(
    #     package="articubot_remote",
    #     executable="move_to_point_action_server.py",
    # )
    task_server_node = Node(
        package="articubot_remote",
        executable="task_server.py",
    )   
    alexa_interface_node = Node(
        package="articubot_remote",
        executable="alexa_interface.py",
    )

    return LaunchDescription([
        task_server_node,
        # alexa_interface_node
    ])
