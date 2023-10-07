from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

  joy_params = os.path.join(get_package_share_directory('nomad_robot'),'config','joystick.yaml')

  joy_node = Node(
    package='joy_linux',
    executable='joy_linux_node',
    parameters=[joy_params],
  )
  
  teleop_node = Node(
    package='teleop_twist_joy', 
    executable='teleop_node',
    name = 'teleop_node',
    parameters=[joy_params],
    remappings=[('/cmd_vel', '/synchro_drive_controller/cmd_vel')]
    )

  return LaunchDescription([
    joy_node,
    teleop_node,    
  ])