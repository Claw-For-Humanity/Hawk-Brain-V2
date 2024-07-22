from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
import yaml
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



# functions
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


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    desc_shared_path = os.path.join(get_package_share_directory('mycobot_description'))
    moveit_shared_path = os.path.join(get_package_share_directory('mycobot_320pi_moveit2'))
    desc_path = os.path.join(desc_shared_path, 'urdf','mycobot_320_pi_2022','mycobot_320_pi_moveit_2022_gz.urdf')
    
    yaml_config = os.path.join(moveit_shared_path,'config', 'ros2_controllers.yaml')


    # ** gazebo **
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])])




    # ** robot_description **
    with open(desc_path,'r') as infp:
        robot_desc = infp.read()

    robot_description = {'robot_description': robot_desc}


    # ** spawn entity **
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'arm_control', '-allow_renaming', 'true'],
    )

    # ** static transformation **
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )



    # ** publish static tf **
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )


    # ** controller_manager **

    controller_manager = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                robot_description,
            {
                "publish_frequency": LaunchConfiguration("publish_frequency"),
            },
                str(yaml_config),
            ],
        )

    # ** joint state publisher ** 
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        remappings=[('/source_list', '/joint_states')]
    )
    
    


    # *********************** MoveIt!2 *********************** #   

    # RVIZ
    rviz_arg = DeclareLaunchArgument(
        "rviz_file", default_value="False", description="Load RVIZ file."
    )

    robot_description_semantic_config = load_file("mycobot_320pi_moveit2", "config/firefighter.srdf")
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config }

    kinematics_yaml = load_yaml("mycobot_320pi_moveit2", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Move group: OMPL Planning.
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }

    ompl_planning_yaml = load_yaml("mycobot_320pi_moveit2", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)


    # MoveIt!2 Controllers:
    moveit_simple_controllers_yaml = load_yaml("mycobot_320pi_moveit2", "config/ros2_controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # START NODE -> MOVE GROUP:
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # RVIZ:
    load_RVIZfile = LaunchConfiguration("rviz_file")
    rviz_base = os.path.join(get_package_share_directory("mycobot_320pi_moveit2"), "config")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    
    rviz_node_full = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
        condition=UnlessCondition(load_RVIZfile),
    )


    # TODO: find out what's going to happen here.
    load_arm_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('mycobot_320pi_moveit2'),'launch'), '/spawn_controllers.launch.py']),
        )


    return LaunchDescription(
        [
            gazebo,
            gz_spawn_entity,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gz_spawn_entity,
                    on_exit=[controller_manager],
                )
            ),
            static_tf,
            node_robot_state_publisher,
            rviz_arg,
            rviz_node_full,
            run_move_group_node,
            load_arm_controller,
            node_joint_state_publisher,
            DeclareLaunchArgument(
                'use_sim_time',
                default_value=use_sim_time,
                description='If true, use simulated clock'),
        ]
    )