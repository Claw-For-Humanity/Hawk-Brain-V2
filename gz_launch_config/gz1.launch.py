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
def generate_demo_launch(moveit_config):

    ######################### Launch Args #########################
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    desc_shared_path = os.path.join(get_package_share_directory('mycobot_description'))
    moveit_shared_path = os.path.join(get_package_share_directory('mycobot_320pi_moveit2'))
    desc_path = os.path.join(desc_shared_path, 'urdf','mycobot_320_pi_2022','mycobot_320_pi_moveit_2022_gz.urdf')
    
    yaml_config = os.path.join(moveit_shared_path,'config', 'ros2_controllers.yaml')

    ld = LaunchDescription()
    
    
    
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

    
    ######################### COMMANDS #########################

    # ** gazebo **
    ld.add_action( 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])])
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
                   )
    
    )







    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = (
        moveit_config.package_path / "launch/static_virtual_joint_tfs.launch.py"
    )
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # Given the published joint states, publish tf for the robot links
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/rsp.launch.py")
            ),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # If database loading was enabled, start mongodb as well
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    # Fake joint driver
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
        )
    )
    
    ld.add_action(
        Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'arm_control', '-allow_renaming', 'true'],
        )
    )

    # gazebo
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/spawn_controllers.launch.py")
            ),
        )
    )

    return ld
