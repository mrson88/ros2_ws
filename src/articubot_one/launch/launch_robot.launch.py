import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node




def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='articubot_one' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )])
    )


    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    


    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])


    ldlidar_node = IncludeLaunchDescription(
      launch_description_source=PythonLaunchDescriptionSource([
          get_package_share_directory('ldlidar_stl_ros2'),
          '/launch/ld06.launch.py'
      ])
  )

    camera_realsense_node=Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            output='screen',

    )
    articubot_recognition=IncludeLaunchDescription(os.path.join(
        get_package_share_directory("articubot_recognition"),
        "launch",
        "launch_yolov8.launch.py"
    ))
    moveit=IncludeLaunchDescription(os.path.join(
        get_package_share_directory("articubot_moveit"),
        "launch",
        "moveit.launch.py"
    ))
    slam_toolbox=IncludeLaunchDescription(os.path.join(
        get_package_share_directory("articubot_one"),
        "launch",
        "online_async_launch.py"
    ))
    navigation_robot=IncludeLaunchDescription(os.path.join(
        get_package_share_directory("articubot_one"),
        "launch",
        "navigation_launch.py"
    ))


    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
    )
    


    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    joint_arm_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["arm_controller"],
    )

    delayed_arm_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_arm_spawner],
        )
    )


    joint_gripper_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["gripper_controller"],
    )

    delayed_gripper_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_gripper_spawner],
        )
    )

    rviz_config = os.path.join(
        get_package_share_directory("articubot_one"),
            "config",
            "main.rviz",
    )
    rviz = Node(
        name='rviz',
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=["-d", rviz_config],
    )
    # Code for delaying a node (I haven't tested how effective it is)
    # 

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        # remappings=[
        #     ('/joint_states', '/joint_commands'),
        # ]
    )


    # Launch them all!



    return LaunchDescription([
        rsp,
        # joint_state_publisher_gui_node,
        # joystick,
        moveit,
        # ldlidar_node,
        # rviz,
        twist_mux,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        delayed_arm_spawner,
        delayed_gripper_spawner,
        # slam_toolbox,
        camera_realsense_node,
        # articubot_recognition,
 
    ])
