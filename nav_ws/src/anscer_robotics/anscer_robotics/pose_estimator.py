from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class PoseEstimator:
    def __init__(self, node: Node):
        self.node = node
        self.publisher = self.node.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

    def set_pose(self, x: float, y: float, yaw: float = 0.0):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.node.get_clock().now().to_msg()

        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0

        import math
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, yaw)
        pose_msg.pose.pose.orientation.x = q[0]
        pose_msg.pose.pose.orientation.y = q[1]
        pose_msg.pose.pose.orientation.z = q[2]
        pose_msg.pose.pose.orientation.w = q[3]

        # Add small covariance to avoid zero matrix
        pose_msg.pose.covariance[0] = 0.25
        pose_msg.pose.covariance[7] = 0.25
        pose_msg.pose.covariance[35] = math.radians(10.0) ** 2

        self.publisher.publish(pose_msg)
        self.node.get_logger().info(
            f"[PoseEstimator] Pose set to x={x}, y={y}, yaw={yaw}")
