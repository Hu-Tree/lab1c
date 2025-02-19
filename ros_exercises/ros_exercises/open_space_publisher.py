import rclpy
import random
import math
from rclpy.node import Node

from std_msgs.msg import Float32
from custom_msgs.msg import OpenSpace
from sensor_msgs.msg import LaserScan


class OpenSpacePublisher(Node):

    def __init__(self):
        super().__init__("open_space_publisher")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_topic_name", "open_space"),
                ("sub_topic_name", "fake_scan"),
            ],
        )
        self.publisher_ = self.create_publisher(
            OpenSpace, self.get_parameter("pub_topic_name").value, 10
        )
        # self.distance_publisher_ = self.create_publisher(
        #     Float32, "open_space/distance", 10
        # )
        # self.angle_publisher_ = self.create_publisher(Float32, "open_space/angle", 10)
        self.subscription = self.create_subscription(
            LaserScan,
            self.get_parameter("sub_topic_name").value,
            self.listener_callback,
            10,
        )

    def listener_callback(self, scan):
        maxDist = scan.range_min
        maxIndex = -1
        for index, dist in enumerate(scan.ranges):
            if dist > maxDist:
                maxIndex, maxDist = index, dist
        msg = OpenSpace()
        msg.distance = maxDist
        msg.angle = maxIndex * scan.angle_increment + scan.angle_min

        self.publisher_.publish(msg)
        self.get_logger().info("Max Distance: %s" % msg.distance)
        # distMsg = Float32()
        # distMsg.data = maxDist
        # self.distance_publisher_.publish(distMsg)

        # angleMsg = Float32()
        # angleMsg.data = maxIndex * scan.angle_increment + scan.angle_min
        # self.angle_publisher_.publish(angleMsg)

        # self.get_logger().info("Max Distance: %s" % distMsg.data)
        # self.get_logger().info("Angle: %s" % angleMsg.data)


def main(args=None):
    rclpy.init(args=args)

    open_space_publisher = OpenSpacePublisher()

    rclpy.spin(open_space_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    open_space_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
