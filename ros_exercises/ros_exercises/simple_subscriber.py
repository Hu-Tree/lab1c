import rclpy
import math
from rclpy.node import Node

from std_msgs.msg import Float32


class SimpleSubscriber(Node):

    def __init__(self):
        super().__init__("simple_subscriber")
        self.subscription = self.create_subscription(
            Float32, "my_random_float", self.listener_callback, 10
        )
        self.publisher_ = self.create_publisher(Float32, "random_float_log", 10)

    #     timer_period = 0.05  # seconds
    #     self.timer = self.create_timer(timer_period, self.timer_callback)

    # def timer_callback(self):
    #     msg = Float32()
    #     msg.data = random.uniform(0, 10)
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.data)

    def listener_callback(self, incoming):
        outgoing = Float32()
        outgoing.data = math.log(incoming.data)
        self.publisher_.publish(outgoing)
        self.get_logger().info('Recieving: "%s"' % incoming.data)
        self.get_logger().info('Publishing: "%s"' % outgoing.data)


def main(args=None):
    rclpy.init(args=args)

    simple_subscriber = SimpleSubscriber()

    rclpy.spin(simple_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simple_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
