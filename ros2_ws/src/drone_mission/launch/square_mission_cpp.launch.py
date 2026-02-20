"""
Launch file untuk Square Mission Node (C++) - ROS 2 Foxy & Humble

Menjalankan square_mission_node (C++) untuk navigasi pola persegi 2x2m.
Kompatibel dengan MAVROS2 + ArduPilot SITL.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_square_size = DeclareLaunchArgument(
        'square_size', default_value='2.0',
        description='Ukuran pola persegi (meter)'
    )
    declare_altitude = DeclareLaunchArgument(
        'altitude', default_value='2.0',
        description='Ketinggian terbang (meter)'
    )
    declare_threshold = DeclareLaunchArgument(
        'threshold', default_value='0.3',
        description='Threshold jarak waypoint (meter)'
    )
    declare_rate = DeclareLaunchArgument(
        'rate', default_value='20',
        description='Frekuensi setpoint (Hz)'
    )

    square_mission_cpp_node = Node(
        package='drone_mission',
        executable='square_mission_node',
        name='square_mission',
        output='screen',
        parameters=[{
            'square_size': LaunchConfiguration('square_size'),
            'altitude': LaunchConfiguration('altitude'),
            'threshold': LaunchConfiguration('threshold'),
            'rate': LaunchConfiguration('rate'),
        }]
    )

    return LaunchDescription([
        declare_square_size,
        declare_altitude,
        declare_threshold,
        declare_rate,
        square_mission_cpp_node,
    ])
