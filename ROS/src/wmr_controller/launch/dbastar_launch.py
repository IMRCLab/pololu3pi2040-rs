import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('wmr_controller')

    default_problem_path = os.path.join(pkg_share, 'external/realtime-dbastar/baselines/wmr-simulator/problems/benchmark/benchmark.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='Pololu09',
            description='Name of the robot in motion capture system'
        ),
        DeclareLaunchArgument(
            #get poses from mocap like in controller interface for pololu
            'mocap_topic',
            default_value='/poses',
            description='Topic for motion capture poses'
        ),
        DeclareLaunchArgument(
            #topic for publishing commands from controller: linear and angular velocity
            'cmd_unicycle_topic',
            default_value='/cmd_unicycle',
            description='Topic to publish unicycle actions (linear and angular velocity)'
        ),
        DeclareLaunchArgument(
            #control loop execution frequency
            'control_dt',
            default_value='0.1', #0.1s in controller simulation code
            description='Control loop rate in s'
        ),
        DeclareLaunchArgument(
            #problem
            'problem',
            default_value=default_problem_path,
            description='Problem'
        ),
        DeclareLaunchArgument(
            'instance',
            default_value='-1_2.2_0.7854_empty',
            description='Name of the benchmark instance to run'
        ),
        DeclareLaunchArgument(
            'obstacle_topic',
            default_value='/obstacles_aabb',
            description='Topic containing the complete current AABB obstacle set'
        ),
        DeclareLaunchArgument(
            'obstacle_change_tolerance',
            default_value='0.02',
            description='AABB coordinate change required to trigger replanning'
        ),
        DeclareLaunchArgument(
            'obstacle_name',
            default_value='Obstacle01',
            description='Name of the obstacle in motion capture system'
        ),
        DeclareLaunchArgument(
            'displacement_thr',
            default_value='0.1',
            description='Difference between predicted and actual pose to trigger displacement replanning'
        ),
        
        Node(
            package='wmr_controller',
            executable='dbastar_controller_node',
            name='dbastar_controller',
            output='screen',
            parameters=[{
                'robot_name': LaunchConfiguration('robot_name'),
                'mocap_topic': LaunchConfiguration('mocap_topic'),
                'cmd_unicycle_topic': LaunchConfiguration('cmd_unicycle_topic'),
                'control_dt': ParameterValue(
                    LaunchConfiguration('control_dt'), value_type=float
                ),
                'problem': LaunchConfiguration('problem'),
                'instance': LaunchConfiguration('instance'),
                'obstacle_topic': LaunchConfiguration('obstacle_topic'),
                'obstacle_change_tolerance': ParameterValue(
                    LaunchConfiguration('obstacle_change_tolerance'), value_type=float
                ),
            }]
        ),

        Node(
            package='wmr_controller',
            executable='obstacle_monitor',
            name='obstacle_monitor',
            output='screen',
            parameters=[{
                'obstacle_name': LaunchConfiguration('obstacle_name'),
                'mocap_topic': LaunchConfiguration('mocap_topic'),
                'obstacle_topic': LaunchConfiguration('obstacle_topic'),
                'obstacle_change_tolerance': ParameterValue(
                    LaunchConfiguration('obstacle_change_tolerance'), value_type=float
                ),
            }]
        )
    ])
