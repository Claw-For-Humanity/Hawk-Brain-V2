import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
import yaml



##################################### LOADING FUNCTIONS #####################################
# load file
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

# load yaml
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None


############################################ LD ############################################
def generate_launch_description():
##################################### PATHS #####################################

  package_name_gz = 'ros_gz_sim'
  package_name_description = 'mycobot_description'
  package_name_mycobot = 'mycobot_320pi_gz'
  default_robot_name = 'mycobot_320pi'

  pkg_ros_gz_sim = FindPackageShare(package=package_name_gz).find(package_name_gz)  
  pkg_share_description = FindPackageShare(package=package_name_description).find(package_name_description)
  pkg_share_mycobot = FindPackageShare(package=package_name_mycobot).find(package_name_mycobot)
  
  gazebo_launch_file_path = 'launch' # FIXME: im new
  gazebo_models_path = 'models'
  rviz_config_file_path = 'config/rviz/moveit.rviz'
  urdf_file_path = 'urdf/xacro/mycobot_320_pi_gz.urdf_v2.xacro'
  

  default_rviz_config_path = os.path.join(pkg_share_mycobot, rviz_config_file_path) # under mycobot
  # default_xacro_urdf_model_path = os.path.join(pkg_share_description, xacro_urdf_file_path) # under desc.
  default_urdf_model_path = os.path.join(pkg_share_description, urdf_file_path)
  gazebo_models_path = os.path.join(pkg_share_mycobot, gazebo_models_path) # [IMPORTANT] gazebo resource path 
  gazebo_launch_file_path = os.path.join(pkg_share_mycobot, gazebo_launch_file_path)   


  # rviz params
  robot_description_semantic_config = load_file("mycobot_320pi_gz", "config/firefighter.srdf")
  robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}
  kinematics_yaml = load_yaml("mycobot_320pi_gz", "config/kinematics.yaml")
  robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
  ompl_planning_pipeline_config = {
      "move_group": {
          "planning_plugin": "ompl_interface/OMPLPlanner",
          "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
          "start_state_max_bounds_error": 0.1,
      }
    }

  ompl_planning_yaml = load_yaml("mycobot_320pi_gz", "config/ompl_planning.yaml")
  ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)
  
  planning_pipelines={'planning_pipelines': ['ompl', 'chomp', 'pilz_industrial_motion_planner'], 'default_planning_pipeline': 'ompl', 'ompl': {'planner_configs': {'SBL': {'type': 'geometric::SBL', 'range': 0.0}, 'EST': {'type': 'geometric::EST', 'range': 0.0, 'goal_bias': 0.05}, 'LBKPIECE': {'type': 'geometric::LBKPIECE', 'range': 0.0, 'border_fraction': 0.9, 'min_valid_path_fraction': 0.5}, 'BKPIECE': {'type': 'geometric::BKPIECE', 'range': 0.0, 'border_fraction': 0.9, 'failed_expansion_score_factor': 0.5, 'min_valid_path_fraction': 0.5}, 'KPIECE': {'type': 'geometric::KPIECE', 'range': 0.0, 'goal_bias': 0.05, 'border_fraction': 0.9, 'failed_expansion_score_factor': 0.5, 'min_valid_path_fraction': 0.5}, 'RRT': {'type': 'geometric::RRT', 'range': 0.0, 'goal_bias': 0.05}, 'RRTConnect': {'type': 'geometric::RRTConnect', 'range': 0.0}, 'RRTstar': {'type': 'geometric::RRTstar', 'range': 0.0, 'goal_bias': 0.05, 'delay_collision_checking': 1}, 'TRRT': {'type': 'geometric::TRRT', 'range': 0.0, 'goal_bias': 0.05, 'max_states_failed': 10, 'temp_change_factor': 2.0, 'min_temperature': '10e-10', 'init_temperature': '10e-6', 'frountier_threshold': 0.0, 'frountierNodeRatio': 0.1, 'k_constant': 0.0}, 'PRM': {'type': 'geometric::PRM', 'max_nearest_neighbors': 10}, 'PRMstar': {'type': 'geometric::PRMstar'}, 'FMT': {'type': 'geometric::FMT', 'num_samples': 1000, 'radius_multiplier': 1.1, 'nearest_k': 1, 'cache_cc': 1, 'heuristics': 0, 'extended_fmt': 1}, 'BFMT': {'type': 'geometric::BFMT', 'num_samples': 1000, 'radius_multiplier': 1.0, 'nearest_k': 1, 'balanced': 0, 'optimality': 1, 'heuristics': 1, 'cache_cc': 1, 'extended_fmt': 1}, 'PDST': {'type': 'geometric::PDST'}, 'STRIDE': {'type': 'geometric::STRIDE', 'range': 0.0, 'goal_bias': 0.05, 'use_projected_distance': 0, 'degree': 16, 'max_degree': 18, 'min_degree': 12, 'max_pts_per_leaf': 6, 'estimated_dimension': 0.0, 'min_valid_path_fraction': 0.2}, 'BiTRRT': {'type': 'geometric::BiTRRT', 'range': 0.0, 'temp_change_factor': 0.1, 'init_temperature': 100, 'frountier_threshold': 0.0, 'frountier_node_ratio': 0.1, 'cost_threshold': '1e300'}, 'LBTRRT': {'type': 'geometric::LBTRRT', 'range': 0.0, 'goal_bias': 0.05, 'epsilon': 0.4}, 'BiEST': {'type': 'geometric::BiEST', 'range': 0.0}, 'ProjEST': {'type': 'geometric::ProjEST', 'range': 0.0, 'goal_bias': 0.05}, 'LazyPRM': {'type': 'geometric::LazyPRM', 'range': 0.0}, 'LazyPRMstar': {'type': 'geometric::LazyPRMstar'}, 'SPARS': {'type': 'geometric::SPARS', 'stretch_factor': 3.0, 'sparse_delta_fraction': 0.25, 'dense_delta_fraction': 0.001, 'max_failures': 1000}, 'SPARStwo': {'type': 'geometric::SPARStwo', 'stretch_factor': 3.0, 'sparse_delta_fraction': 0.25, 'dense_delta_fraction': 0.001, 'max_failures': 5000}},'arm_group': {'default_planner_config': 'None', 'planner_configs': ['SBL', 'EST', 'LBKPIECE', 'BKPIECE', 'KPIECE', 'RRT', 'RRTConnect', 'RRTstar', 'TRRT', 'PRM', 'PRMstar', 'FMT', 'BFMT', 'PDST', 'STRIDE', 'BiTRRT', 'LBTRRT', 'BiEST', 'ProjEST', 'LazyPRM', 'LazyPRMstar', 'SPARS', 'SPARStwo'], 'projection_evaluator': 'joints(joint2_to_joint1,joint3_to_joint2)', 'longest_valid_segment_fraction': 0.005}}, 'chomp': {'planning_plugin': 'chomp_interface/CHOMPPlanner', 'enable_failure_recovery': True, 'jiggle_fraction': 0.05, 'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints', 'ridge_factor': 0.01, 'start_state_max_bounds_error': 0.1}, 'pilz_industrial_motion_planner': {'planning_plugin': 'pilz_industrial_motion_planner/CommandPlanner', 'request_adapters': '', 'default_planner_config': 'PTP', 'capabilities': 'pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService'}}



##################################### LAUNCH CONFIGs #####################################   
  # Launch configuration variables specific to simulation
  robot_name = LaunchConfiguration('robot_name')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  urdf_model = LaunchConfiguration('urdf_model')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_rviz = LaunchConfiguration('use_rviz')
  use_sim_time = LaunchConfiguration('use_sim_time')
  
  # Set the default pose
  x = LaunchConfiguration('x')
  y = LaunchConfiguration('y')
  z = LaunchConfiguration('z')
  roll = LaunchConfiguration('roll')
  pitch = LaunchConfiguration('pitch')
  yaw = LaunchConfiguration('yaw')

  # for movegroup (TODO: implement movegroup later.)
  # should_publish = LaunchConfiguration("publish_monitored_planning_scene")
  # monitor_dynamics = LaunchConfiguration("monitor_dynamics")




##################################### LAUNCH ARGs ##################################### 
  # Declare the launch arguments  
  declare_robot_name_cmd = DeclareLaunchArgument(
    name='robot_name',
    default_value=default_robot_name,
    description='The name for the robot')

  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

  declare_urdf_model_path_cmd = DeclareLaunchArgument(
    name='urdf_model', 
    default_value=default_urdf_model_path, # NOTE: xacro was here 
    description='Absolute path to robot urdf file') 
    
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')

  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')
    
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')

  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start Gazebo')

  declare_x_cmd = DeclareLaunchArgument(
    name='x',
    default_value='0.0',
    description='x component of initial position, meters')

  declare_y_cmd = DeclareLaunchArgument(
    name='y',
    default_value='0.0',
    description='y component of initial position, meters')
    
  declare_z_cmd = DeclareLaunchArgument(
    name='z',
    default_value='0.05',
    description='z component of initial position, meters')
    
  declare_roll_cmd = DeclareLaunchArgument(
    name='roll',
    default_value='0.0',
    description='roll angle of initial orientation, radians')

  declare_pitch_cmd = DeclareLaunchArgument(
    name='pitch',
    default_value='0.0',
    description='pitch angle of initial orientation, radians')

  declare_yaw_cmd = DeclareLaunchArgument(
    name='yaw',
    default_value='0.0',
    description='yaw angle of initial orientation, radians')
  
  # from here is for launch_move_group NOTE: work on movegroup later.
  # declare_debug_cmd = DeclareBooleanLaunchArg(
  #   "debug",
  #   default_value=False)
  
  # declare_allow_trajectory_execution_cmd = DeclareBooleanLaunchArg(
  #   "allow_trajectory_execution",
  #  default_value=True)

  # declare_publish_monitored_planning_scene_cmd = DeclareBooleanLaunchArg(
  #     "publish_monitored_planning_scene",
  #     default_value=True)

  # declare_capabilities_cmd = DeclareLaunchArgument(
  #   "capabilities",
  #   default_value="")

  # declare_disable_capabilities_cmd = DeclareLaunchArgument(
  #   "disable_capabilities",
  #   default_value="")

  # declare_monitor_dynamics_cmd = DeclareBooleanLaunchArg(
  #   "monitor_dynamics",
  #   default_value=False)


##################################### CONFIGS ##################################### 
  # NOTE: work on movegroup later
  # move_group_configuration = {
  #       "publish_robot_description_semantic": True,
  #       "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
  #       # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
  #       "capabilities": ParameterValue(
  #           LaunchConfiguration("capabilities"), value_type=str
  #       ),
  #       "disable_capabilities": ParameterValue(
  #           LaunchConfiguration("disable_capabilities"), value_type=str
  #       ),
  #       # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
  #       "publish_planning_scene": should_publish,
  #       "publish_geometry_updates": should_publish,
  #       "publish_state_updates": should_publish,
  #       "publish_transforms_updates": should_publish,
  #       "monitor_dynamics": monitor_dynamics, # (if) FIXME: set this to False
  #       "use_sim_time": use_sim_time,
  #   }


##################################### ACTION ##################################### 
  # Specify the actions
  set_env_vars_resources = AppendEnvironmentVariable(
    'GZ_SIM_RESOURCE_PATH',
    gazebo_models_path)

  # Start arm controller
  start_arm_controller_cmd = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
        'arm_controller'],
        output='screen')

  # Start Gazebo environment
  start_gazebo_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    launch_arguments=[('gz_args', [' -r -v4 ', 'empty.sdf'])])

  # Launch joint state broadcaster
  start_joint_state_broadcaster_cmd = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
        'joint_state_broadcaster'],
        output='screen')

  # define robot desc.
  robot_description_content = ParameterValue(Command(['xacro ', urdf_model]), value_type=str)
  robot_description = {'robot_description': robot_description_content}
  #  Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='both',
    parameters=[{
      'use_sim_time': use_sim_time, 
      'robot_description': robot_description_content}])

  # start rviz
  sim_time = {'use_sim_time': use_sim_time}
  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=["-d", rviz_config_file],
    parameters=[
      robot_description,
      robot_description_semantic,
      robot_description_kinematics,
      sim_time,
      planning_pipelines # or planning_pipelines      
      ] 
    )  
    
  # Spawn the robot (NOTE: deleted '-string', robot_description_content)
  start_gazebo_ros_spawner_cmd = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
      '-name', robot_name,
      '-topic', 'robot_description',
      '-x', x,
      '-y', y,
      '-z', z,
      '-R', roll,
      '-P', pitch,
      '-Y', yaw
      ],
    output='screen')
  
  # Launch the joint state broadcaster after spawning the robot
  load_joint_state_broadcaster_cmd = RegisterEventHandler(
     event_handler=OnProcessExit(
     target_action=start_gazebo_ros_spawner_cmd ,
     on_exit=[start_joint_state_broadcaster_cmd],))

  # Launch the arm controller after launching the joint state broadcaster
  load_arm_controller_cmd = RegisterEventHandler(
    event_handler=OnProcessExit(
    target_action=start_joint_state_broadcaster_cmd,
    on_exit=[start_arm_controller_cmd],))

  # Start arm controller
  start_arm_controller_cmd = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
        'arm_controller'],
        output='screen')

  
  # TODO: finish this (not even sure if we need this or not)
  spawn_controllers =  (
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('mycobot_320pi_gz'),'launch'), '/spawn_controllers.launch.py']),
        )
    )

  # NOTE: later thing
  # start_move_group = Node(
  #     package="moveit_ros_move_group",
  #     executable="move_group",
  #     # commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"), # im not sure if this exists in our package
  #     output="screen",
  #     parameters=[
  #       robot_description, # originally. moveit_config.to_dict()
  #       robot_description_semantic,
  #       robot_description_kinematics,
  #       ompl_planning_yaml,
  #       sim_time,
  #       move_group_configuration 
  #       ],
  #     # extra_debug_args=["--debug"],
  #     # Set the display variable, in case OpenGL code is used internally
  #     # additional_env={"DISPLAY": ":0"}
  # )



  
  # NOTE: LATERRRR
  # load_move_group_cmd = RegisterEventHandler(
  #     event_handler=OnProcessExit(
  #         target_action=start_move_group,
  #         on_exit=[start_rviz_cmd]
  #     )
  # )

  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_robot_name_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_urdf_model_path_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)

  ld.add_action(declare_x_cmd)
  ld.add_action(declare_y_cmd)
  ld.add_action(declare_z_cmd)
  ld.add_action(declare_roll_cmd)
  ld.add_action(declare_pitch_cmd)
  ld.add_action(declare_yaw_cmd) 
  
  # NOTE: MOVE GROUP INTEGRATION FOR LATER.
  # ld.add_action(declare_debug_cmd)
  # ld.add_action(declare_allow_trajectory_execution_cmd)
  # ld.add_action(declare_publish_monitored_planning_scene_cmd)
  # ld.add_action(declare_capabilities_cmd)
  # ld.add_action(declare_disable_capabilities_cmd)
  # ld.add_action(declare_monitor_dynamics_cmd)

  # Add any actions
  ld.add_action(set_env_vars_resources)
  ld.add_action(start_gazebo_cmd)
  ld.add_action(start_robot_state_publisher_cmd)
  
  ld.add_action(start_rviz_cmd)
  
  ld.add_action(start_gazebo_ros_spawner_cmd)
  
  # orig: joint_state -> arm_controller 
  ld.add_action(load_joint_state_broadcaster_cmd)
  ld.add_action(load_arm_controller_cmd)


  return ld