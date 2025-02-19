import rclpy
import random
import math
from rclpy.node import Node

from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan


class FakeScanPublisher(Node):

    def __init__(self):
        super().__init__("fake_scan_publisher")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("topic_name", "fake_scan"),
                ("pub_rate", 20),
                ("angle_min", (-2 / 3) * math.pi),
                ("angle_max", (2 / 3) * math.pi),
                ("range_min", 1.0),
                ("range_max", 10.0),
                ("angle_increment", (1 / 300) * math.pi),
            ],
        )
        self.laser_publisher_ = self.create_publisher(
            LaserScan, self.get_parameter("topic_name").value, 10
        )
        self.range_publisher_ = self.create_publisher(Float32, "range_test", 10)
        timer_period = 1.0 / self.get_parameter("pub_rate").value  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = LaserScan()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle_min = self.get_parameter("angle_min").value
        msg.angle_max = self.get_parameter("angle_max").value
        msg.angle_increment = self.get_parameter("angle_increment").value
        msg.scan_time = 1.0 / self.get_parameter("pub_rate").value
        msg.range_min = self.get_parameter("range_min").value
        msg.range_max = self.get_parameter("range_max").value
        msg.ranges = [
            random.uniform(msg.range_min, msg.range_max)
            for _ in range(
                int((msg.angle_max - msg.angle_min) / msg.angle_increment + 1)
            )
        ]
        self.laser_publisher_.publish(msg)
        rangesLength = Float32()
        rangesLength.data = float(
            (msg.angle_max - msg.angle_min) / msg.angle_increment + 1
        )
        self.range_publisher_.publish(rangesLength)

        self.get_logger().info("Ranges length: %s" % rangesLength.data)
        self.get_logger().info("Angle increment: %s" % msg.angle_increment)


def main(args=None):
    rclpy.init(args=args)

    fake_scan_publisher = FakeScanPublisher()

    rclpy.spin(fake_scan_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fake_scan_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
