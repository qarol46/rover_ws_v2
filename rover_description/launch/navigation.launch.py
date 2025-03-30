import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Пути к файлам
    pkg_dir = get_package_share_directory('rover_description')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    map_yaml = os.path.join(pkg_dir, 'maps', 'map.yaml')

    return LaunchDescription([
        # Запуск Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('nav2_bringup'), '/launch/navigation_launch.py']
            ),
            launch_arguments={
                'params_file': nav2_params,
                'map': map_yaml,
                'use_sim_time': 'true'
            }.items()
        ),

        # Автоматическая активация всех компонентов Nav2
        ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/amcl', 'configure'],
            shell=True
        ),
        ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/amcl', 'activate'],
            shell=True
        )
    ])