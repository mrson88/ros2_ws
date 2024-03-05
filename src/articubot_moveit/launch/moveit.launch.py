import os
import yaml
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.conditions import IfCondition

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
    declared_arguments = []
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

    ompl_planning_pipeline_config = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }

    ompl_planning_pipeline_config["ompl"].update(ompl_config)
    moveit_cpp_pnp_yaml_file_name = (
        get_package_share_directory("articubot_core") + "/config/moveit_cpp_pnp.yaml"
    )

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
        # remappings=[('/robot_description','/arm_robot')]
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
    arm_pnp_as = Node(
            name='articubot_node',
            package='articubot_remote',
            executable='task_server_node',
            output='screen',
            parameters=[
                {
                    'base_frame': 'odom',
                    'robot_description_semantic': robot_description_semantic,
                    'robot_description_kinematics': kinematics_config,
                    # 'robot_description_planning' : joint_limits_config,
                    'planning_pipelines': ['ompl'],
                    'ompl': ompl_config,
                    'robot_description': robot_description,
                },
                # moveit_cpp_config,
                # ompl_planning_pipeline_config,
                
                # moveit_cpp_pnp_yaml_file_name,
                moveit_controllers,
                trajectory_execution,
                planning_scene_monitor_config,
            ],
            # condition=IfCondition(use_pnp)
        )
    
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "use_pnp",
    #         default_value="true",
    #         description="Start pnp server if it is required.",
    #     )
    # ) 
    # use_pnp = LaunchConfiguration("use_pnp")


    arm_pnp_as_pnp = Node(
            name='innobot_node',
            package='articubot_core',
            executable='innobot_node',
            output='screen',
            parameters=[
                {
                    'base_frame': 'world',
                    'robot_description_semantic': robot_description_semantic,
                    'robot_description_kinematics': kinematics_config,
                    # 'robot_description_planning' : joint_limits_config,
                    'planning_pipelines': ['ompl'],
                    'ompl': ompl_config,
                    'robot_description': robot_description,
                },
                # moveit_cpp_config,
                # ompl_planning_pipeline_config,
                
                # moveit_cpp_pnp_yaml_file_name,
                moveit_controllers,
                trajectory_execution,
                planning_scene_monitor_config,
            ],
            # condition=IfCondition(use_pnp)
        )
    
    arm_pnp_as_pnp_1 = Node(
            name='innobot_node_1',
            package='articubot_remote',
            executable='pick_n_place_node',
            output='screen',
            parameters=[
                {
                    'base_frame': 'world',
                    'robot_description_semantic': robot_description_semantic,
                    'robot_description_kinematics': kinematics_config,
                    # 'robot_description_planning' : joint_limits_config,
                    'planning_pipelines': ['ompl'],
                    'ompl': ompl_config,
                    'robot_description': robot_description,
                },
                # moveit_cpp_config,
                # ompl_planning_pipeline_config,
                
                # moveit_cpp_pnp_yaml_file_name,
                moveit_controllers,
                trajectory_execution,
                planning_scene_monitor_config,
            ],
            # condition=IfCondition(use_pnp)
        )

    return LaunchDescription([
        move_group_node,
        rviz,
        arm_pnp_as,
        # arm_pnp_as_pnp,
        # arm_pnp_as_pnp_1,
        
        ]

    )