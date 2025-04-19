import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from anscer_pkgs.action import NavigateWormhole
from geometry_msgs.msg import PoseStamped


class TestClient(Node):
    def __init__(self):
        super().__init__('test_navigator_client')
        self._action_client = ActionClient(
            self, NavigateWormhole, 'navigate_wormhole')

    def send_goal(self):
        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()

        goal_msg = NavigateWormhole.Goal()

        # Define the goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = -4.0
        goal_pose.pose.position.y = 1.0
        goal_pose.pose.orientation.w = 1.0

        goal_msg.target_pose = goal_pose
        goal_msg.target_map = 'map1'  # Different from default "room1"

        self.get_logger().info(
            f"Sending goal to map '{goal_msg.target_map}'...")
        self._send_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Feedback: {feedback.percent_complete:.1f}% complete")

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f"Result: success={result.success}, message='{result.message}'")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TestClient()
    node.send_goal()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
