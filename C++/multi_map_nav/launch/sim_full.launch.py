from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('multi_map_nav')

    # 1) TurtleBot3 in Gazebo
    tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch', 'turtlebot3_world.launch.py'
            )
        )
    )

    # 2) Nav2 bringup (AMCL + map_server params come from your nav2_params.yaml)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'bringup_launch.py'
            )
        ),
        launch_arguments={
            # ← You must supply this:
            'map': os.path.join(pkg, 'maps', 'map1.yaml'),

            # your existing args:
            'use_sim_time': 'true',
            'autostart':     'true',
            'params_file':   os.path.join(pkg, 'config', 'nav2_params.yaml'),
        }.items()
    )

    # 3) Start with map1
    load_map1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg, 'launch', 'load_map1.launch.py'))
    )

    # 4) Your action server node
    action_server = Node(
        package='multi_map_nav',
        executable='navigate_to_map_action_server',
        name='navigate_to_map_action_server',
        output='screen'
    )

    # 5) RViz, pre‑loaded with our config
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg, 'rviz', 'sim_config.rviz')]
    )

    return LaunchDescription([
        # tb3,
        nav2, load_map1,
        action_server,
        rviz
    ])
