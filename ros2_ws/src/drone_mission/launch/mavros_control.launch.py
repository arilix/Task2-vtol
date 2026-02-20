"""
Launch file untuk MAVROS Control Node (Python) - ROS 2 Foxy & Humble

Menjalankan mavros_control.py dengan opsi konfigurasi melalui launch arguments.
Kompatibel dengan MAVROS2 + ArduPilot SITL.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ---- Launch Arguments ----
    declare_fcu_url = DeclareLaunchArgument(
        'fcu_url', default_value='tcp://127.0.0.1:5762',
        description='FCU connection URL untuk ArduPilot SITL'
    )
    declare_mission_type = DeclareLaunchArgument(
        'mission_type', default_value='hover',
        description='Tipe misi: hover, square, circle, waypoints'
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
    declare_hover_duration = DeclareLaunchArgument(
        'hover_duration', default_value='10.0',
        description='Durasi hover (detik)'
    )
    declare_square_size = DeclareLaunchArgument(
        'square_size', default_value='2.0',
        description='Ukuran pola persegi (meter)'
    )
    declare_circle_radius = DeclareLaunchArgument(
        'circle_radius', default_value='2.0',
        description='Radius lingkaran (meter)'
    )
    declare_circle_points = DeclareLaunchArgument(
        'circle_points', default_value='36',
        description='Jumlah titik dalam lingkaran'
    )
    declare_launch_mavros = DeclareLaunchArgument(
        'launch_mavros', default_value='true',
        description='Apakah ikut menjalankan MAVROS node'
    )

    # ---- MAVROS Control Node (Python) ----
    mavros_control_node = Node(
        package='drone_mission',
        executable='mavros_control.py',
        name='mavros_control',
        output='screen',
        parameters=[{
            'mission_type': LaunchConfiguration('mission_type'),
            'altitude': LaunchConfiguration('altitude'),
            'threshold': LaunchConfiguration('threshold'),
            'rate': LaunchConfiguration('rate'),
            'hover_duration': LaunchConfiguration('hover_duration'),
            'square_size': LaunchConfiguration('square_size'),
            'circle_radius': LaunchConfiguration('circle_radius'),
            'circle_points': LaunchConfiguration('circle_points'),
        }]
    )

    return LaunchDescription([
        declare_fcu_url,
        declare_mission_type,
        declare_altitude,
        declare_threshold,
        declare_rate,
        declare_hover_duration,
        declare_square_size,
        declare_circle_radius,
        declare_circle_points,
        declare_launch_mavros,
        mavros_control_node,
    ])
