from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class WormholeManager:
    def __init__(self, node: Node) -> None:
        self.node: Node = node
        self.wormholes = {
            ("map1", "map2"): PoseStamped(),
            ("map2", "map1"): PoseStamped(),
        }

        self._init_wormhole_poses()

    def _init_wormhole_poses(self) -> None:
        self._extract_map(
            -7.0, -2.0, 'map1', 'map2'
        )
        self._extract_map(
            7.0, -4.0, 'map2', 'map1'
        )

    # TODO Rename this here and in `_init_wormhole_poses`
    def _extract_map(self, x_cord: float, y_cord: float, first: str, second: str) -> None:
        result = PoseStamped()
        result.header.frame_id = "map"
        result.pose.position.x = x_cord
        result.pose.position.y = y_cord
        result.pose.orientation.w = 1.0
        self.wormholes[first, second] = result

    def get_wormhole_pose(self, from_map: str, to_map: str) -> PoseStamped | None:
        key: tuple[str, str] = (from_map, to_map)
        if key in self.wormholes:
            self.node.get_logger().info(
                f"[WormholeManager] Found wormhole: {key}")
            return self.wormholes[key]
        else:
            self.node.get_logger().warn(
                f"[WormholeManager] No wormhole defined between '{from_map}' and '{to_map}'")
            return None
