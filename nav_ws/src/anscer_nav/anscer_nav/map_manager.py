import rclpy
from rclpy import Future
from rclpy.client import Client
from rclpy.node import Node
from nav2_msgs.srv import LoadMap
from ament_index_python.packages import get_package_share_directory
import os


class MapManager(Node):
    def __init__(self, map_list: list[str]) -> None:
        super().__init__(node_name='map_manager', namespace='anscer_robotics')

        self.__map_list: list[str] = map_list
        self.__current_map: str = 'map1'

        self.__load_map: Client = self.create_client(
            srv_type=LoadMap,
            srv_name='/map_server/load_map'
        )

        self.wait_for_service(
            client=self.__load_map,
            service_name='/map_server/load_map'
        )

        self.log('Map manager online')

    @property
    def current_map(self) -> str:
        return self.__current_map

    @current_map.setter
    def current_map(self, map_name: str) -> None:
        if map_name in self.__map_list:
            self.__current_map = map_name
        else:
            raise ValueError('Unknown map name')

    def log(self, msg: str) -> None:
        self.get_logger().info(f'[MapManager] {msg}')

    def wait_for_service(self, client, service_name) -> None:
        while not client.wait_for_service(timeout_sec=2.0):
            self.log(f'Waiting for {service_name} service...')

    def get_map_path(self, map_name: str) -> str:
        current_package: str = get_package_share_directory('anscer_nav')
        return os.path.join(current_package, 'maps', f'{map_name}.yaml')

    def get_coordinates(self, map_name: str) -> tuple[float, float]:
        return (-7, -2) if map_name == 'map1' else (7, -4)

    def switch_map(self, target_map: str) -> bool:
        if target_map == self.__current_map:
            self.log(f'Already on {target_map}')
            return True

        map_url: str = self.get_map_path(target_map)
        self.log(f'Map path: {map_url}')

        self.log(f'Loading new map: {target_map}')

        request = LoadMap.Request()
        request.map_url = map_url
        future: Future = self.__load_map.call_async(request)
        rclpy.spin_until_future_complete(node=self, future=future)

        result = future.result()

        if result and result.result == 0:
            self.log('Map loaded successfully')

            self.__current_map = target_map
            return True

        self.log(
            f'Failed to load map. Result code: {result.result if result else "None"}')
        return False
