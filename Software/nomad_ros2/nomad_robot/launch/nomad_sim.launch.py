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

from launch_ros.actions import Node

import xacro


def generate_launch_description():
  # Launch gazebo simulation
  gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
  )
  
  # Load robot description
  pkg_path = os.path.join(get_package_share_directory('nomad_robot'))
  xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
  robot_description_config = xacro.process_file(xacro_file)
  params = {'robot_description': robot_description_config.toxml()}

  # Create robot_state_publisher
  node_robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[params]
  )

  # spawn gazebo entity
  spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
    arguments=['-topic', 'robot_description','-entity', 'tricycle'],
    output='screen')

  # Load state broadcaster controller
  load_joint_state_controller = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','joint_state_broadcaster'],
    output='screen'
  )

  # load synchro driver controller
  load_tricycle_controller = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'synchro_drive_controller'],
    output='screen'
  )

  # Load RVIZ2
  rviz = Node(
    package="rviz2",
    executable="rviz2",
    arguments=[
      "-d",
      os.path.join(pkg_path, "config/config.rviz"),
    ],
    output="screen",
  )
  
  # Load joy controller
  joy = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('nomad_robot'), 'launch', 'joy_controller.launch.py')]),
  )

  return LaunchDescription([
    RegisterEventHandler(
      event_handler=OnProcessExit(
      target_action=spawn_entity,
      on_exit=[load_joint_state_controller],)
    ),
    RegisterEventHandler(
      event_handler=OnProcessExit(
      target_action=load_joint_state_controller,
      on_exit=[load_tricycle_controller],)
    ),
    gazebo,
    rviz,
    node_robot_state_publisher,
    spawn_entity,
    joy,])