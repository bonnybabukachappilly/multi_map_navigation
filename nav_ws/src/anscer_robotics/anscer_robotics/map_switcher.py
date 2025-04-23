from nav2_msgs.srv import LoadMap
import rclpy
from rclpy.client import Client
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from anscer_robotics.manager import WormholeManager
import os


class MapSwitcher:
    """Switching the map in Rviz by traveling to the wormhole location

    Attributes
    ----------
    node: Node
        ROS node for creating all clients
    
    _current_map: str
        current map name
    
    _database: WormholeManager
        Getting wormhole locations
    
    map_service_client: Client
        Service client to load new map in to rviz

    Methods
    -------
    @property
    current_map(self) -> str:
        Returns the current map

    switch_map(self, target_map: str) -> bool:
        Handles switching the map by navigating to the wormhole

    get_map_path(self, map_name: str) -> str:
        Get the location of the map

    get_coordinates(self, map_name: str) -> tuple[float, float]:
        get coordinate of gazebo locations for wormholes
    """

    __slots__: list[str] = [
        'node',
        '_current_map',
        '_database',
        'map_service_client'
    ]

    def __init__(self, node: Node, db: WormholeManager) -> None:
        self.node: Node = node
        self._current_map: str = "map1"
        self._database: WormholeManager = db

        self.node.get_logger().info(
            f"[MapSwitcher] Initial map: {self._current_map}")

        self.map_service_client: Client = node.create_client(
            LoadMap, '/map_server/load_map')

        while not self.map_service_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().info('Waiting for "/map_server/load_map" service...')

    @property
    def current_map(self) -> str:
        """Returns the current map

        Returns
        -------
        str
            current map name
        """
        return self._current_map

    def switch_map(self, target_map: str) -> bool:
        """Handles switching the map by navigating to the wormhole

        Parameters
        ----------
        target_map : str
            Map to navigate to

        Returns
        -------
        bool
            Status of the map switch
        """
        if target_map == self._current_map:
            self.node.get_logger().info(
                f"[MapSwitcher] Already on map '{target_map}'")
            return True

        map_yaml_path = self.get_map_path(target_map)
        self.node.get_logger().info(
            f"[MapSwitcher] Map YAML path: {map_yaml_path}")
        self.node.get_logger().info(
            f"[MapSwitcher] Switching from '{self._current_map}' to '{target_map}'")

        self.node.get_logger().info(f'Loading new map: {map_yaml_path}')
        load_req = LoadMap.Request()
        load_req.map_url = map_yaml_path
        load_future = self.map_service_client.call_async(load_req)
        rclpy.spin_until_future_complete(self.node, load_future)

        result = load_future.result()
        self.node.get_logger().info(f'Result: {result}')

        if result and result.result == 0:
            self.node.get_logger().info(
                '[MapSwitcher] Map loaded successfully.')

            self._current_map = target_map

            return True
        else:
            self.node.get_logger().error(
                f'[MapSwitcher] Failed to load map. Result code: {result.result if result else "None"}')
            return False

    def get_map_path(self, map_name: str) -> str:
        """Get the location of the map

        Parameters
        ----------
        map_name : str
            Name of the map for which location is required

        Returns
        -------
        str
            location of map
        """
        current_package: str = get_package_share_directory('anscer_robotics')
        return os.path.join(current_package, 'maps', f'{map_name}.yaml')

    def get_coordinates(self, map_name: str) -> tuple[float, float]:
        """get coordinate of gazebo locations for wormholes

        Parameters
        ----------
        map_name : str
            target map name

        Returns
        -------
        tuple[float, float]
            gazebo coordinates
        """

        coordinates = self._database.get_wormhole_pose(
            from_map=self._current_map, to_map=map_name)

        return (
            coordinates.pose.position.x,  # type: ignore
            coordinates.pose.position.y  # type: ignore
        )
