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
            #get poses from mocap like in controller interface for pololu
            'mocap_topic',
            default_value='/poses',
            description='Topic for motion capture poses'
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
