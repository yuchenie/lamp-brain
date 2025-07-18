from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick_node',
            output='screen',
            parameters=[{
                'use_mag': True,
                'publish_tf': False,
                'world_frame': 'odom',
                'base_frame': 'base_link',
                'reverse_mag_z': False
            }],
            remappings=[
                ('imu/data_raw', '/imu/data_raw'),
                ('imu/mag', '/imu/mag'),
                ('imu/data', '/imu/data')  # Output topic (fused IMU)
            ]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'frequency': 10.0,
                'sensor_timeout': 0.1,

                'imu0': '/imu/data',
                'imu0_config': [
                    False, False, False,  # Position
                    True, True, True,     # Orientation (roll, pitch, yaw)
                    False, False, False,  # Velocity
                    False, False, False,  # Angular velocity
                    False, False, False   # Linear acceleration
                ],
                'imu0_differential': False,
                'imu0_remove_gravitational_acceleration': False,
                'imu0_queue_size': 10,

                'world_frame': 'odom',
                'base_link_frame': 'base_link',
                'odom_frame': 'odom',
                'publish_tf': True,
            }]
        )
    ])
