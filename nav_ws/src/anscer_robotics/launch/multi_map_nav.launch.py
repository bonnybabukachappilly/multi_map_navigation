from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    map_name = LaunchConfiguration('map_name').perform(context)

    # Get paths
    pkg_tb3_nav2 = get_package_share_directory('turtlebot3_navigation2')
    pkg_this = get_package_share_directory('anscer_robotics')
    
    print(map_name)

    # Choose the map
    map_path = os.path.join(pkg_this, 'maps', f'{map_name}.yaml')

    # Launch Navigation2 with the selected map
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_nav2, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
            'map': map_path,
            'use_sim_time': 'true'
        }.items()
    )

    return [nav2_launch]


def generate_launch_description():
    declare_map_arg = DeclareLaunchArgument(
        'map_name',
        default_value='map1',
        description='Choose which map to load: map1 or map2'
    )

    return LaunchDescription([
        declare_map_arg,
        OpaqueFunction(function=launch_setup)
    ])
