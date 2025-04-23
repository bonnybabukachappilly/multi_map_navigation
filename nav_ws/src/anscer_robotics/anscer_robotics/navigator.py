import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped


class Navigator:
    """Navigation request is handled here
    
    Attributes
    ----------
    node: Node
        ROS node for creating all clients
        
    navigator: BasicNavigator
        Nav2 Navigation

    Methods
    -------
    go_to_pose(self, pose: PoseStamped) -> bool:
        Navigate to the requested pose
    """
    
    __slots__: list[str] = [
        'node',
        'navigator'
    ]

    def __init__(self, node: Node) -> None:
        self.node: Node = node
        self.navigator = BasicNavigator()

        self.node.get_logger().info("[Navigator] Initialized")

    def go_to_pose(self, pose: PoseStamped) -> bool:
        """Navigate to the requested pose

        Parameters
        ----------
        pose : PoseStamped
            Location details as PoseStamped message

        Returns
        -------
        bool
            Status of the navigation
        """
        self.node.get_logger().info(
            f"[Navigator] Navigating to pose: {pose.pose.position}")
        self.navigator.goToPose(pose)

        while not self.navigator.isTaskComplete():
            if feedback := self.navigator.getFeedback():
                self.node.get_logger().info(
                    f"[Navigator] Distance remaining: {feedback.distance_remaining:.2f}")
            rclpy.spin_once(self.node, timeout_sec=0.1)

        result = self.navigator.getResult()

        if result.value == 1:
            self.node.get_logger().info("[Navigator] Goal succeeded!")
            return True
        else:
            self.node.get_logger().warn(
                f"[Navigator] Goal failed with status: {result}")
            return False
