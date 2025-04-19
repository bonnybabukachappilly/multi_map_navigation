import os
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import DeleteEntity, SpawnEntity

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchService

class TeleportBot:
    def __init__(self, node: Node, robot_name: str = 'burger'):
        self.node = node
        self.robot_name = robot_name

        self.delete_client = self.node.create_client(
            DeleteEntity, '/delete_entity')
        self.spawn_client = self.node.create_client(
            SpawnEntity, '/spawn_entity')

        self._wait_for_services()

    def _wait_for_services(self):
        self.node.get_logger().info('Waiting for Gazebo services...')
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for /delete_entity...')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for /spawn_entity...')
        self.node.get_logger().info('Gazebo services ready.')

    def teleport(self, x: float, y: float):
        self.node.get_logger().info(
            f'Teleporting {self.robot_name} to x={x}, y={y}')
        self._delete_entity()
        self._spawn_entity(x, y)
        return True

    def _delete_entity(self):
        req = DeleteEntity.Request()
        req.name = self.robot_name
        future = self.delete_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result():
            self.node.get_logger().info(
                f'Successfully deleted {self.robot_name}')
        else:
            self.node.get_logger().warn(
                f'Failed to delete {self.robot_name}. It may not exist.')

    def _spawn_entity(self, x: float, y: float):
        launch_file_dir = os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'launch')

        ld = LaunchDescription()

        spawn_turtlebot_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': str(x),
                'y_pose': str(y)
            }.items()
        )

        ld.add_action(spawn_turtlebot_cmd)
        ls = LaunchService()
        ls.include_launch_description(ld)
        ls.run()
