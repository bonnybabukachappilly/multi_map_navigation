import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from anscer_pkgs.action import NavigateWormhole


from anscer_robotics.navigator import Navigator
from anscer_robotics.map_switcher import MapSwitcher
from anscer_robotics.manager import WormholeManager
from anscer_robotics.teleport_gazebo import TeleportBot
from anscer_robotics.pose_estimator import PoseEstimator


class WormholeNavigator(Node):
    def __init__(self) -> None:
        super().__init__('wormhole_navigator')

        self.navigator = Navigator(self)
        self.map_switcher = MapSwitcher(self)
        self.wormhole_manager = WormholeManager(self)
        self.teleport = TeleportBot(self)
        self.pose_estimator = PoseEstimator(self)

        self._action_server = ActionServer(
            self,
            NavigateWormhole,
            'navigate_wormhole',
            self.execute_callback
        )

        self.get_logger().info("[WormholeNavigator] Ready to accept goals")

    def set_initial_pos(self) -> None:
        self.teleport_robot('map1', 'map2')

    def execute_callback(self, goal_handle) -> NavigateWormhole.Result:
        self.get_logger().info("[Action] Received navigation request")

        target_pose = goal_handle.request.target_pose
        target_map = goal_handle.request.target_map.strip()

        current_map = self.map_switcher.get_current_map()

        feedback_msg = NavigateWormhole.Feedback()
        feedback_msg.percent_complete = 0.0
        goal_handle.publish_feedback(feedback_msg)

        if current_map != target_map:
            self.get_logger().info(
                f"[Action] Current map '{current_map}' != target map '{target_map}'")

            wormhole_pose = self.wormhole_manager.get_wormhole_pose(
                current_map, target_map)
            if wormhole_pose is None:
                goal_handle.abort()
                return NavigateWormhole.Result(success=False, message="No wormhole available")

            feedback_msg.percent_complete = 30.0
            goal_handle.publish_feedback(feedback_msg)

            success: bool = self.navigator.go_to_pose(wormhole_pose)
            if not success:
                goal_handle.abort()
                return NavigateWormhole.Result(success=False, message="Failed to reach wormhole")

            self.map_switcher.switch_map(target_map)

            self.teleport_robot(target_map, current_map)
            feedback_msg.percent_complete = 60.0
            goal_handle.publish_feedback(feedback_msg)

        # Final navigation
        success = self.navigator.go_to_pose(target_pose)
        feedback_msg.percent_complete = 100.0
        goal_handle.publish_feedback(feedback_msg)

        if success:
            goal_handle.succeed()
            return NavigateWormhole.Result(success=True, message="Goal reached successfully")
        else:
            goal_handle.abort()
            return NavigateWormhole.Result(success=False, message="Failed to reach final goal")

    def teleport_robot(self, arg0, arg1) -> None:
        teleport_pose = self.wormhole_manager.get_wormhole_pose(arg0, arg1)
        self.teleport.teleport(
            teleport_pose.pose.position.x, teleport_pose.pose.position.y
        )
        self.pose_estimator.set_pose(
            teleport_pose.pose.position.x, teleport_pose.pose.position.y
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WormholeNavigator()
    node.set_initial_pos()
    rclpy.spin(node)
    print("DONE")
