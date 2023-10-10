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
  package_name = "nomad_robot"
  
  # Get Robot State publisher launcher
  robot_state_pub = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [os.path.join(get_package_share_directory(package_name), "launch", "rsp.launch.py")]),
    launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
  )
  
  # Get robot description from state publisher node
  robot_description = Command(["ros2 param get --hide-type /robot_state_publisher robot_description"])
  

  
  # Get controller param file
  controller_params_file = os.path.join(get_package_share_directory(package_name), "config", "nomad_drive_controller.yaml")
  
  # Launch controller manager node
  controller_manager = Node(
    package = "controller_manager",
    executable = "ros2_control_node",
    parameters = [{"robot_description":robot_description}, controller_params_file],
  )
  
  delayed_controller_manager = TimerAction(period=0.1, actions=[controller_manager])
  
  # Spawn controllers in the control manager
  
  joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
  )

  robot_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["synchro_drive_controller", "--controller-manager", "/controller_manager"],
  )
  
  # Delay the spwaners until the controller manager is completely loaded 
  delayed_state_publisher = RegisterEventHandler(
    event_handler = OnProcessStart(
      target_action = controller_manager,
      on_start = [joint_state_broadcaster_spawner]
    )
  )
  
  delayed_controller = RegisterEventHandler(
    event_handler = OnProcessStart(
      target_action = joint_state_broadcaster_spawner,
      on_start = [robot_controller_spawner]
    )
  )
  
  # Load RVIZ2
  rviz = Node(
    package="rviz2",
    executable="rviz2",
    output="screen",
  )
  
  return LaunchDescription([
    robot_state_pub,
    delayed_controller_manager,
    delayed_state_publisher,
    delayed_controller,
    rviz
  ])