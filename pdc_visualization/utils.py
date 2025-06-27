import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from tf2_ros import Buffer, TransformListener, LookupException, TimeoutException, ExtrapolationException
from geometry_msgs.msg import TransformStamped

class TFHandler:
    def __init__(self, node: Node):
        self.node = node
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)

        # We use an executor to manually spin the node in spin_once().
        # This is necessary because we need to process incoming TF messages
        # while waiting in a loop for the transform to become available.
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(node)

    def get_transform(self, from_frame: str, to_frame: str, timeout_sec: float = 5.0) -> TransformStamped:
        start_time = self.node.get_clock().now()
        transform = None

        while (self.node.get_clock().now() - start_time).nanoseconds * 1e-9 < timeout_sec:
            try:
                transform = self.buffer.lookup_transform(
                    to_frame,
                    from_frame,
                    rclpy.time.Time(),  # latest available
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                self.node.get_logger().info(
                    f"Got transform from '{from_frame}' to '{to_frame}'"
                )
                break  # success
            except (LookupException, TimeoutException, ExtrapolationException) as e:
                self.executor.spin_once(timeout_sec=0.1)

        if transform is None:
            self.node.get_logger().error(
                f"Could not get transform from '{from_frame}' to '{to_frame}' within {timeout_sec} seconds."
            )

        return transform
