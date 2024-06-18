
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
# import xacro

def generate_launch_description():

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])])


    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # doc = xacro.parse(open("/home/cfh/ntest/src/mycobot_description/urdf/mycobot_320_pi_2022/mycobot_320.sdf"))
    # params = {'robot_description':doc.toxml()}
    
    robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                PathJoinSubstitution(
                    [FindPackageShare('mycobot_description'),
                    'urdf', 'mycobot_320_pi_2022', 'mycobot_320pi.sdf']
                ),
            ]
        )
    
    robot_description = {'robot_description': robot_description_content}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'six_dof_arm', '-allow_renaming', 'true'],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_controller'],
        output='screen'
    )

    return LaunchDescription([
        # Launch gazebo environment
        gazebo,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_gripper_controller],
            )
        ),
        node_robot_state_publisher,
        gz_spawn_entity,
        # # Launch Arguments
        # DeclareLaunchArgument(
        #     'use_sim_time',
        #     default_value=use_sim_time,
        #     description='If true, use simulated clock'),
    ])