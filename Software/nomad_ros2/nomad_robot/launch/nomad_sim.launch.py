# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import Node

import xacro


def generate_launch_description():
  
  package_name = "nomad_robot"
  
  # Get Robot State publisher launcher
  robot_state_pub = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [os.path.join(get_package_share_directory(package_name), "launch", "rsp.launch.py")]),
    launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'false'}.items()
  )
  
  # spawn gazebo entity
  spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
    arguments=['-topic', 'robot_description','-entity', 'nomad'],
    output='screen')
  
  # Controller loaders
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
    event_handler=OnProcessExit(
      target_action=spawn_entity,
      on_exit=[joint_state_broadcaster_spawner],
    )
  )
  
  delayed_controller = RegisterEventHandler(
    event_handler=OnProcessExit(
      target_action=joint_state_broadcaster_spawner,
      on_exit=[robot_controller_spawner],
    )
  )
  
  # Launch gazebo simulation
  gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
  )
  
  # Load RVIZ2
  rviz = Node(
    package="rviz2",
    executable="rviz2",
    arguments=[
      "-d",
      os.path.join(get_package_share_directory(package_name), "config/config.rviz"),
    ],
    output="screen",
  )
  
  # Load joy controller
  joy = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory(package_name), 'launch', 'joy_controller.launch.py')]),
  )

  return LaunchDescription([
    gazebo,
    #rviz,
    robot_state_pub,
    spawn_entity,
    delayed_state_publisher,
    delayed_controller,
    #joy,
  ])