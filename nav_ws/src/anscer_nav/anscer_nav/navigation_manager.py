import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped


class NavigationManager(Node):
    def __init__(self) -> None:
        super().__init__(node_name='navigation_manager', namespace='anscer_robotics')
        self.navigator = BasicNavigator()

        self.log("Navigation manager online")
        
    def log(self, msg: str) -> None:
        self.get_logger().info(f'[NavigationManager] {msg}')

    def go_to_pose(self, pose: PoseStamped) -> bool:
        self.log(
            f"[Navigator] Navigating to pose: {pose.pose.position}")
        self.navigator.goToPose(pose)

        while not self.navigator.isTaskComplete():
            if feedback := self.navigator.getFeedback():
                self.log(
                    f"[Navigator] Distance remaining: {feedback.distance_remaining:.2f}")
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.navigator.getResult()

        if result.value == 1:  # SUCCEEDED
            self.log("[Navigator] Goal succeeded!")
            return True
        else:
            self.get_logger().warn(
                f"[Navigator] Goal failed with status: {result}")
            return False
