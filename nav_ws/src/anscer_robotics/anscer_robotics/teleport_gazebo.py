import os
import rclpy
from rclpy import Future
from rclpy.client import Client
from rclpy.node import Node
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from typing import Literal
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchService


class TeleportBot:
    """Used to teleport robot in Gazebo environment

    Attributes
    ----------
    node: Node
        ROS node for creating all clients

    robot_name: str
        Name of the robot used in Gazebo

    delete_client: Client
        Service client to delete the existing robot

    spawn_client: Client
        Service client to spawn the robot in desired location

    Methods
    ----------
    _wait_for_services(self) -> None:
        Waiting for each service to be online

    teleport(self, x: float, y: float) -> Literal[True]:
        Teleport the robot to a desired location

    _delete_entity(self) -> None:
        Delete existing robot in the gazebo

    _spawn_entity(self, x: float, y: float) -> None:
        Spawn a new robot in specified location
    """

    __slots__: list[str] = [
        'node',
        'robot_name',
        'delete_client',
        'spawn_client',
    ]

    def __init__(self, node: Node, robot_name: str = 'burger') -> None:
        self.node: Node = node
        self.robot_name: str = robot_name

        self.delete_client: Client = self.node.create_client(
            DeleteEntity, '/delete_entity')
        self.spawn_client: Client = self.node.create_client(
            SpawnEntity, '/spawn_entity')

        self._wait_for_services()

    def _wait_for_services(self) -> None:
        """Waiting for each service to be online"""
        self.node.get_logger().info('Waiting for Gazebo services...')

        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for /delete_entity...')

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for /spawn_entity...')

        self.node.get_logger().info('Gazebo services ready.')

    def teleport(self, x: float, y: float) -> Literal[True]:
        """Teleport the robot to a desired location

        Parameters
        ----------
        x : float
            X Coordinates
        y : float
            Y Coordinates

        Returns
        -------
        Literal[True]
            Teleported
        """
        self.node.get_logger().info(
            f'Teleporting {self.robot_name} to x={x}, y={y}')
        self._delete_entity()
        self._spawn_entity(x, y)
        return True

    def _delete_entity(self) -> None:
        """Delete existing robot in the gazebo"""
        req = DeleteEntity.Request()
        req.name = self.robot_name
        future: Future = self.delete_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result():
            self.node.get_logger().info(
                f'Successfully deleted {self.robot_name}')
        else:
            self.node.get_logger().warn(
                f'Failed to delete {self.robot_name}. It may not exist.')

    def _spawn_entity(self, x: float, y: float) -> None:
        """Spawn a new robot in specified location

        Parameters
        ----------
        x : float
            X Coordinates
        y : float
            Y Coordinates
        """
        launch_file_dir: str = os.path.join(
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
