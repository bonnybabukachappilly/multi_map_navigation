import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from anscer_pkgs.action import NavigateWormhole
from anscer_nav.map_manager import MapManager
from anscer_nav.wormhole_manager import WormholeManager


class WormholeNavigation(Node):
    def __init_(self) -> None:
        super().__init__(node_name='navigation_manager', namespace='anscer_robotics')

        _ = ActionServer(
            node=self,
            action_type=NavigateWormhole,
            action_name='navigate_multi_room',
            execute_callback=self.__callback
        )

        self.__map = MapManager(map_list=['map1', 'map2'])
        self.__wormhole = WormholeManager(node=self)

        self.log('Wormhole navigation online')

    def log(self, msg: str) -> None:
        self.get_logger().info(f'[WormholeNavigation] {msg}')

    def __callback(self, goal) -> None:
        self.log('Goal received')

        target_pose = goal.request.target_pose
        target_map = goal.request.target_map.strip()

        current_map = self.__map.current_map

        feedback_msg = NavigateWormhole.Feedback()
        feedback_msg.percent_complete = 0.0
        goal.publish_feedback(feedback_msg)

        if current_map != target_map:
            self.log(
                f"Current map '{current_map}' != target map '{target_map}'")
            
            location = self.__wormhole.get_wormhole_pose(
                current_map, target_map
            )
            
            if location is None:
                goal.abort()
                return NavigateWormhole.Result(success=False, message="No wormhole available")

            feedback_msg.percent_complete = 30.0
            goal.publish_feedback(feedback_msg)