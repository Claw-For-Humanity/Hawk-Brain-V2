from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from srdfdom.srdf import SRDF

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare
import os
import yaml


# FUNCTIONS
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None


# DEMO LAUNCH
def generate_launch_description():

    ######################### Launch Args #########################
    ld = LaunchDescription()
    # ld.add_action(DeclareLaunchArgument('use_sim_time', default=True))
    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="15.0"))
    desc_shared_path = os.path.join(get_package_share_directory('mycobot_description'))
    moveit_shared_path = os.path.join(get_package_share_directory('mycobot_320pi_moveit2'))
    desc_path = os.path.join(desc_shared_path, 'urdf','mycobot_320_pi_2022','mycobot_320_pi_moveit_2022_gz.urdf')
    
    
    
    ######################### BOOLEANS ######################### 
    # debugging booleans
    ld.add_action(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )

    # use rviz boolean
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

    
    ######################### ACTIONS #########################

    # ** gazebo **
    ld.add_action( 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution(
                    [FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])
                                       ]
                                       ),
            launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])]),
    )

    # ** robot_description **
    with open(desc_path,'r') as infp:
        robot_desc = infp.read()

    robot_description = {'robot_description': robot_desc}

    # ** spawn on gazebo **
    ld.add_action(
        Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'arm_control', '-allow_renaming', 'true'],
                   ),
    
    )

    # ** publish tf2 - for gz **
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            output="log",
            arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
            ),
    )

    # ** publish robot state - for gz **
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name='robot_state_publisher',
            output="both",
            parameters=[robot_description],
        ),
    )

    # publish tf for the robot links - for rviz
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(f"{moveit_shared_path}/launch/rsp.launch.py")
            ),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(f"{moveit_shared_path}/launch/move_group.launch.py")
            ),
        ),
    )

    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(f"{moveit_shared_path}/launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    )

    # If database loading was enabled, start mongodb as well
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(f"{moveit_shared_path}/launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        ),
    )

    # Controller Manager -> fake control.
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                robot_description,
                str(f"{moveit_shared_path}/config/ros2_controllers.yaml"),
            ],
        ),
    )

    # ** there is no controllers currently so this will be useless **
    # ld.add_action(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             str(f"{moveit_shared_path}/launch/spawn_controllers.launch.py")
    #         ),
    #     )
    # )

    ld.add_action(
        Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        remappings=[('/source_list', '/joint_states')]
        )
    )

    return ld
