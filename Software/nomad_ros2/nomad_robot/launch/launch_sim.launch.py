import os
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description() :
  """ Launch gazebo simulation-

  Returns:
      LaunchDescription: launch description.
  """
  
  package_name = "nomad_robot"
  # Launch robot_state_publisher (rsp)
  rsp = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory(package_name),'launch','rsp.launch.py')]),
    launch_arguments={'use_sim_time': 'true'}.items())

  # Include the Gazebo launch file, provided by the gazebo_ros package
  gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),)

  # Run the spawner node from the gazebo_ros package.
  spawn_entity = Node(
    package = 'gazebo_ros', 
    executable = 'spawn_entity.py',
    arguments = ['-topic', 'robot_description','-entity', 'nomad_robot'],
    output = 'screen'
  )
  
  rviz = Node(
    package ="rviz2",
    executable = "rviz2",
  )

  # Launch them all!
  return LaunchDescription([
    rsp,
    gazebo,
    spawn_entity,
    rviz
  ])