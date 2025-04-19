from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.srv import LoadMap
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import os


class MapSwitcher:
    def __init__(self, node: Node) -> None:
        self.node: Node = node
        self.current_map: str = "map1"
        self.publisher: Publisher = node.create_publisher(
            String, "/current_map", 10)
        self.node.get_logger().info(
            f"[MapSwitcher] Initial map: {self.current_map}")

        self.map_service_client = node.create_client(
            LoadMap, '/map_server/load_map')
        self.deactivate_client = node.create_client(
            ChangeState, '/map_server/change_state')
        self.activate_client = node.create_client(
            ChangeState, '/map_server/change_state')

        # Wait for services
        self.wait_for_service(self.map_service_client, '/map_server/load_map')
        self.wait_for_service(
            self.deactivate_client,
            '/map_server/change_state'
        )
        self.wait_for_service(self.activate_client, '/map_server/change_state')

    def wait_for_service(self, client, service_name):
        while not client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().info(
                f'Waiting for {service_name} service...')

    def switch_map(self, target_map: str) -> bool:
        if target_map == self.current_map:
            self.node.get_logger().info(
                f"[MapSwitcher] Already on map '{target_map}'")
            return True

        map_yaml_path = self.get_map_path(target_map)
        self.node.get_logger().info(
            f"[MapSwitcher] Map YAML path: {map_yaml_path}")
        self.node.get_logger().info(
            f"[MapSwitcher] Switching from '{self.current_map}' to '{target_map}'")

        # Deactivate map server
        # self.node.get_logger().info('Deactivating map_server...')
        # deactivate_req = ChangeState.Request()
        # deactivate_req.transition.id = Transition.TRANSITION_DEACTIVATE
        # deactivate_future = self.deactivate_client.call_async(deactivate_req)
        # rclpy.spin_until_future_complete(self.node, deactivate_future)
        # result = deactivate_future.result()
        # self.node.get_logger().info(f'Result: {result}')

        # Load the new map
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

            # Activate map server
            # self.node.get_logger().info('Reactivating map_server...')
            # activate_req = ChangeState.Request()
            # activate_req.transition.id = Transition.TRANSITION_ACTIVATE
            # activate_future = self.activate_client.call_async(activate_req)
            # rclpy.spin_until_future_complete(self.node, activate_future)

            # Publish new map name
            self.current_map = target_map
            self.publisher.publish(String(data=self.current_map))

            return True
        else:
            self.node.get_logger().error(
                f'[MapSwitcher] Failed to load map. Result code: {result.result if result else "None"}')
            return False

    def get_current_map(self) -> str:
        return self.current_map

    def get_map_path(self, map_name: str) -> str:
        current_package: str = get_package_share_directory('anscer_robotics')
        return os.path.join(current_package, 'maps', f'{map_name}.yaml')

    def get_coordinates(self, map_name: str) -> tuple[float, float]:
        return (-7, -2) if map_name == 'map1' else (7, -4)
