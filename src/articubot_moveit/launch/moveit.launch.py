import os
import yaml
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def load_file(file_path):
    """Load the contents of a file into a string"""
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def run_xacro(xacro_file):
    """Run xacro and output a file in the same directory with the same name, w/o a .xacro suffix"""
    urdf_file, ext = os.path.splitext(xacro_file)
    if ext != '.xacro':
        raise RuntimeError(f'Input file to xacro must have a .xacro extension, got {xacro_file}')
    os.system(f'xacro {xacro_file} -o {urdf_file}')
    return urdf_file


def generate_launch_description():
    xacro_file = get_package_file('articubot_one', 'description/robot.urdf.xacro')
    urdf_file = run_xacro(xacro_file)
    srdf_file = get_package_file('articubot_moveit', 'config/articubot.srdf')
    kinematics_file = get_package_file('articubot_moveit', 'config/kinematics.yaml')
    ompl_config_file = get_package_file('articubot_moveit', 'config/ompl_planning.yaml')
    moveit_controllers_file = get_package_file('articubot_moveit', 'config/moveit_controllers.yaml')

    robot_description = load_file(urdf_file)
    robot_description_semantic = load_file(srdf_file)
    kinematics_config = load_yaml(kinematics_file)
    ompl_config = load_yaml(ompl_config_file)

    moveit_controllers = {
        'moveit_simple_controller_manager' : load_yaml(moveit_controllers_file),
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    }
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01
    }
    planning_scene_monitor_config = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True
    }

    # MoveIt node
    move_group_node = Node(
        name="move_group_arm",
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_config,
                'ompl': ompl_config,
                'planning_pipelines': ['ompl'],
                
            },
            {'publish_robot_description_semantic': True},
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor_config,
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Visualization (parameters needed for MoveIt display plugin)
    rviz_config = os.path.join(
        get_package_share_directory("articubot_moveit"),
            "config",
            "moveit1.rviz",
    )
    rviz = Node(
        name='rviz',
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=["-d", rviz_config],
        parameters=[
            {
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_config,
            }
        ],
        
    )


    return LaunchDescription([
        move_group_node,
        rviz,
        ]

    )


# --------------------------------

# import os
# from launch import LaunchDescription
# from moveit_configs_utils import MoveItConfigsBuilder
# from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from ament_index_python.packages import get_package_share_directory


# def generate_launch_description():

#     is_sim = LaunchConfiguration('is_sim')
    
#     is_sim_arg = DeclareLaunchArgument(
#         'is_sim',
#         default_value='True'
#     )

#     moveit_config = (
#         MoveItConfigsBuilder("robot", package_name="arduinobot_moveit")
#         .robot_description(file_path=os.path.join(
#             get_package_share_directory("articubot_one"),
#             "urdf",
#             "robot.urdf.xacro"
#             )
#         )
#         .robot_description_semantic(file_path="config/articubot.srdf")
#         .trajectory_execution(file_path="config/moveit_controllers.yaml")
#         .to_moveit_configs()
#     )

#     move_group_node = Node(
#         package="moveit_ros_move_group",
#         executable="move_group",
#         output="screen",
#         parameters=[moveit_config.to_dict(), 
#                     {'use_sim_time': is_sim},
#                     {'publish_robot_description_semantic': True}],
#         arguments=["--ros-args", "--log-level", "info"],
#     )

#     # RViz
#     rviz_config = os.path.join(
#         get_package_share_directory("articubot_moveit"),
#             "config",
#             "moveit1.rviz",
#     )
#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         output="log",
#         arguments=["-d", rviz_config],
#         parameters=[
#             moveit_config.robot_description,
#             moveit_config.robot_description_semantic,
#             moveit_config.robot_description_kinematics,
#             moveit_config.joint_limits,
#         ],
#     )

#     return LaunchDescription(
#         [
#             is_sim_arg,
#             move_group_node, 
#             rviz_node
#         ]
#     )