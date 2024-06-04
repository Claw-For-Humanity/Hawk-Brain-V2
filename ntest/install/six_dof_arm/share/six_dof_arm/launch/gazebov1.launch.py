import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import (OnProcessStart, OnProcessExit)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ign_ros2_control'),'launch','gazebo.launch.py')])
    )
    print(f"\ngazebo is {gazebo}")
    
    xacro_file = "/home/cfh/ntest/src/mycobot_description/urdf/mycobot_320_pi_2022/mycobot_320_pi_2022.urdf"
    
    print(f'xacro file package path is {xacro_file}\n')
    

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description':doc.toxml()}
    print(f'params is {params}')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output = 'screen',
        parameters=[params]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic','/robot_description',
                                   '-entity','six_dof_arm'],
                                   output='screen')
    
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])