from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    pkg_tb3_nav2: str = get_package_share_directory('turtlebot3_navigation2')
    pkg_this: str = get_package_share_directory('anscer_robotics')

    map_path: str = os.path.join(pkg_this, 'maps', 'map1.yaml')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_nav2, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
            'map': map_path,
            'use_sim_time': 'true'
        }.items()
    )

    wormhole_navigator_node = Node(
        package='anscer_robotics',
        executable='navigator',
        name='wormhole_navigator',
        output='screen'
    )

    return LaunchDescription([
        nav2_launch,
        wormhole_navigator_node
    ])
