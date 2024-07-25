import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
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

  pkg_ros_gz_sim = FindPackageShare(package=package_name_gz).find(package_name_gz)  
  pkg_share_description = FindPackageShare(package=package_name_description).find(package_name_description)
  pkg_share_mycobot = FindPackageShare(package=package_name_mycobot).find(package_name_mycobot)


  default_robot_name = 'mycobot_320pi'
  
  gazebo_models_path = 'models' # leave as is
  ros_gz_bridge_config_file_path = 'config/ros_gz_bridge.yaml'
  rviz_config_file_path = 'config/rviz/mycobot_320_pi.rviz'
  urdf_file_path = 'urdf/xacro/mycobot_320_pi_gz.urdf.xacro'


  default_ros_gz_bridge_config_file_path = os.path.join(pkg_share_mycobot, ros_gz_bridge_config_file_path)
  default_rviz_config_path = os.path.join(package_name_mycobot, rviz_config_file_path) # under mycobot
  default_urdf_model_path = os.path.join(pkg_share_description, urdf_file_path) # under desc.
  gazebo_models_path = os.path.join(pkg_share_mycobot, gazebo_models_path) # [IMPORTANT] gazebo resource path 


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
  # planning_pipelines={'planning_pipelines': ['ompl', 'chomp', 'pilz_industrial_motion_planner'], 'default_planning_pipeline': 'ompl', 'ompl': {'planner_configs': {'SBL': {'type': 'geometric::SBL', 'range': 0.0}, 'EST': {'type': 'geometric::EST', 'range': 0.0, 'goal_bias': 0.05}, 'LBKPIECE': {'type': 'geometric::LBKPIECE', 'range': 0.0, 'border_fraction': 0.9, 'min_valid_path_fraction': 0.5}, 'BKPIECE': {'type': 'geometric::BKPIECE', 'range': 0.0, 'border_fraction': 0.9, 'failed_expansion_score_factor': 0.5, 'min_valid_path_fraction': 0.5}, 'KPIECE': {'type': 'geometric::KPIECE', 'range': 0.0, 'goal_bias': 0.05, 'border_fraction': 0.9, 'failed_expansion_score_factor': 0.5, 'min_valid_path_fraction': 0.5}, 'RRT': {'type': 'geometric::RRT', 'range': 0.0, 'goal_bias': 0.05}, 'RRTConnect': {'type': 'geometric::RRTConnect', 'range': 0.0}, 'RRTstar': {'type': 'geometric::RRTstar', 'range': 0.0, 'goal_bias': 0.05, 'delay_collision_checking': 1}, 'TRRT': {'type': 'geometric::TRRT', 'range': 0.0, 'goal_bias': 0.05, 'max_states_failed': 10, 'temp_change_factor': 2.0, 'min_temperature': '10e-10', 'init_temperature': '10e-6', 'frountier_threshold': 0.0, 'frountierNodeRatio': 0.1, 'k_constant': 0.0}, 'PRM': {'type': 'geometric::PRM', 'max_nearest_neighbors': 10}, 'PRMstar': {'type': 'geometric::PRMstar'}, 'FMT': {'type': 'geometric::FMT', 'num_samples': 1000, 'radius_multiplier': 1.1, 'nearest_k': 1, 'cache_cc': 1, 'heuristics': 0, 'extended_fmt': 1}, 'BFMT': {'type': 'geometric::BFMT', 'num_samples': 1000, 'radius_multiplier': 1.0, 'nearest_k': 1, 'balanced': 0, 'optimality': 1, 'heuristics': 1, 'cache_cc': 1, 'extended_fmt': 1}, 'PDST': {'type': 'geometric::PDST'}, 'STRIDE': {'type': 'geometric::STRIDE', 'range': 0.0, 'goal_bias': 0.05, 'use_projected_distance': 0, 'degree': 16, 'max_degree': 18, 'min_degree': 12, 'max_pts_per_leaf': 6, 'estimated_dimension': 0.0, 'min_valid_path_fraction': 0.2}, 'BiTRRT': {'type': 'geometric::BiTRRT', 'range': 0.0, 'temp_change_factor': 0.1, 'init_temperature': 100, 'frountier_threshold': 0.0, 'frountier_node_ratio': 0.1, 'cost_threshold': '1e300'}, 'LBTRRT': {'type': 'geometric::LBTRRT', 'range': 0.0, 'goal_bias': 0.05, 'epsilon': 0.4}, 'BiEST': {'type': 'geometric::BiEST', 'range': 0.0}, 'ProjEST': {'type': 'geometric::ProjEST', 'range': 0.0, 'goal_bias': 0.05}, 'LazyPRM': {'type': 'geometric::LazyPRM', 'range': 0.0}, 'LazyPRMstar': {'type': 'geometric::LazyPRMstar'}, 'SPARS': {'type': 'geometric::SPARS', 'stretch_factor': 3.0, 'sparse_delta_fraction': 0.25, 'dense_delta_fraction': 0.001, 'max_failures': 1000}, 'SPARStwo': {'type': 'geometric::SPARStwo', 'stretch_factor': 3.0, 'sparse_delta_fraction': 0.25, 'dense_delta_fraction': 0.001, 'max_failures': 5000}},'arm_group': {'default_planner_config': 'None', 'planner_configs': ['SBL', 'EST', 'LBKPIECE', 'BKPIECE', 'KPIECE', 'RRT', 'RRTConnect', 'RRTstar', 'TRRT', 'PRM', 'PRMstar', 'FMT', 'BFMT', 'PDST', 'STRIDE', 'BiTRRT', 'LBTRRT', 'BiEST', 'ProjEST', 'LazyPRM', 'LazyPRMstar', 'SPARS', 'SPARStwo'], 'projection_evaluator': 'joints(joint2_to_joint1,joint3_to_joint2)', 'longest_valid_segment_fraction': 0.005}}, 'chomp': {'planning_plugin': 'chomp_interface/CHOMPPlanner', 'enable_failure_recovery': True, 'jiggle_fraction': 0.05, 'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints', 'ridge_factor': 0.01, 'start_state_max_bounds_error': 0.1}, 'pilz_industrial_motion_planner': {'planning_plugin': 'pilz_industrial_motion_planner/CommandPlanner', 'request_adapters': '', 'default_planner_config': 'PTP', 'capabilities': 'pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService'}}



##################################### LAUNCH CONFIGs #####################################   
  # Launch configuration variables specific to simulation
  headless = LaunchConfiguration('headless')
  robot_name = LaunchConfiguration('robot_name')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  urdf_model = LaunchConfiguration('urdf_model')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_rviz = LaunchConfiguration('use_rviz')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  
  # Set the default pose
  x = LaunchConfiguration('x')
  y = LaunchConfiguration('y')
  z = LaunchConfiguration('z')
  roll = LaunchConfiguration('roll')
  pitch = LaunchConfiguration('pitch')
  yaw = LaunchConfiguration('yaw')




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

  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Display the Gazebo GUI if False, otherwise run in headless mode')

  declare_urdf_model_path_cmd = DeclareLaunchArgument(
    name='urdf_model', 
    default_value=default_urdf_model_path, 
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
    default_value='true',
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




##################################### ACTION ##################################### 

  # Specify the actions
  set_env_vars_resources = AppendEnvironmentVariable(
    'GZ_SIM_RESOURCE_PATH',
    gazebo_models_path)
  
  # Start Gazebo server (var world -> 'empty.sdf')
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'gz_args': ['-r -s -v4 ', 'empty.sdf'], 'on_exit_shutdown': 'true'}.items())

  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    launch_arguments={'gz_args': '-g -v4 '}.items(),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
    
  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  robot_description_content = ParameterValue(Command(['xacro ', urdf_model]), value_type=str)
  robot_description = {'robot_description': robot_description_content}
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{
      'use_sim_time': use_sim_time, 
      'robot_description': robot_description_content}])

  # Launch RViz
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
      ompl_planning_pipeline_config,
      robot_description_kinematics      
    ])  
    
  # Spawn the robot
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
    
  # Bridge ROS topics and Gazebo messages for establishing communication
  start_gazebo_ros_bridge_cmd = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    parameters=[{
      'config_file': default_ros_gz_bridge_config_file_path,
    }],
    output='screen'
  )  
    
  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_robot_name_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_simulator_cmd)
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

  # Add any actions
  ld.add_action(set_env_vars_resources)
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
  ld.add_action(start_robot_state_publisher_cmd)
  
  ld.add_action(start_gazebo_ros_spawner_cmd)
  ld.add_action(start_gazebo_ros_bridge_cmd)
  ld.add_action(start_rviz_cmd)
  
  return ld