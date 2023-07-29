import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.descriptions import ParameterValue 
namespace = 'nomad200'
use_sim_time = True

def generate_launch_description():
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg = get_package_share_directory('nomad200_sim')
    path_to_urdf = pkg+'/urdf/nomad200/robot.urdf.xacro'
    
    ign_gazebo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
      os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
    )


    state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{
        'robot_description': ParameterValue(Command(['xacro ', str(path_to_urdf)]),
                                       value_type=str)}])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.join(pkg, 'rviz', 'robot_description_publisher.rviz')
        ]
    )

    # clock bridge
    clock_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                        namespace=namespace,
                        name='clock_bridge',
                        output='screen',
                        arguments=['/clock' + '@rosgraph_msgs/msg/Clock' + '[ignition.msgs.Clock'],
                        )

    # cmd_vel bridge
    cmd_vel_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                          namespace=namespace,
                          name='cmd_vel_bridge',
                          output='screen',
                          parameters=[{
                              'use_sim_time': use_sim_time
                          }],
                          arguments=[
                              '/cmd_vel' + '@geometry_msgs/msg/Twist' + ']ignition.msgs.Twist'
                          ],
                          remappings=[
                              ('/cmd_vel', '/cmd_vel')
                          ])

    # odometry bridge
    odometry_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                           namespace=namespace,
                           name='odometry_bridge',
                           output='screen',
                           parameters=[{
                               'use_sim_time': use_sim_time
                           }],
                           arguments=[
                               '/model/my_custom_model' + '/odometry' +
                               '@nav_msgs/msg/Odometry' + '[ignition.msgs.Odometry'
                           ],
                           remappings=[
                               ('/model/my_custom_model' + '/odometry', '/odom')
                           ])

    
    # odom to base_link transform bridge
    odom_base_tf_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                               namespace=namespace,
                               name='odom_base_tf_bridge',
                               output='screen',
                               parameters=[{
                                   'use_sim_time': use_sim_time
                               }],
                               arguments=[
                                   '/model/my_custom_model' + '/tf' +
                                   '@tf2_msgs/msg/TFMessage' + '[ignition.msgs.Pose_V'
                               ],
                               remappings=[
                                   ('/model/my_custom_model' + '/tf', '/tf')
                               ])

    # Spawn
    spawn = Node(package='ros_ign_gazebo', executable='create',
                 arguments=[
                    '-name', 'my_custom_model',
                    '-x', '1.2',
                    '-z', '0.5',
                    '-Y', '3.4',
                    '-topic', '/robot_description'],
                 output='screen')


    # joint state bridge
    joint_state_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                              namespace=namespace,
                              name='joint_state_bridge',
                              output='screen',
                              parameters=[{
                                  'use_sim_time': use_sim_time
                              }],
                              arguments=[
                                  '/world/empty/model/my_custom_model/joint_state'
                                  + '@sensor_msgs/msg/JointState' + '[ignition.msgs.Model'
                              ],
                              remappings=[
                                  ('/world/empty/model/my_custom_model/joint_state', '/joint_states')
                              ])


    # imu bridge
    imu_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                          namespace=namespace,
                          name='imu_bridge',
                          output='screen',
                          parameters=[{
                              'use_sim_time': use_sim_time
                          }],
                          arguments=[
                              '/imu' + '@sensor_msgs/msg/Imu' + '[ignition.msgs.IMU'
                          ],
                          remappings=[
                              ('/imu', '/imu')
                          ])


    return LaunchDescription([
      DeclareLaunchArgument(
          'ign_args',
            default_value=[os.path.join(pkg, 'worlds', 'empty.sdf')]),
        ign_gazebo,
        spawn,
        rviz,
        state_publisher,
        clock_bridge,
        odometry_bridge,
        cmd_vel_bridge,
        odom_base_tf_bridge,
        joint_state_bridge,
        imu_bridge
    ])