import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class StaticTFCamPublisher(Node):
    def __init__(self):
        super().__init__("static_tf_cam_publisher")
        self.static_broadcaster = StaticTransformBroadcaster(self)

        left_tf = TransformStamped()
        left_tf.header.stamp = self.get_clock().now().to_msg()
        left_tf.header.frame_id = "base_link"
        left_tf.child_frame_id = "left_cam"

        left_tf.transform.translation.x = 0.0
        left_tf.transform.translation.y = 0.05
        left_tf.transform.translation.z = 0.0

        left_tf.transform.rotation.x = 0.0
        left_tf.transform.rotation.y = 0.0
        left_tf.transform.rotation.z = 0.0
        left_tf.transform.rotation.w = 1.0

        right_tf = TransformStamped()
        right_tf.header.stamp = self.get_clock().now().to_msg()
        right_tf.header.frame_id = "left_cam"
        right_tf.child_frame_id = "right_cam"

        right_tf.transform.translation.x = 0.0
        right_tf.transform.translation.y = -0.1
        right_tf.transform.translation.z = 0.0

        right_tf.transform.rotation.x = 0.0
        right_tf.transform.rotation.y = 0.0
        right_tf.transform.rotation.z = 0.0
        right_tf.transform.rotation.w = 1.0

        self.static_broadcaster.sendTransform([left_tf, right_tf])
        self.get_logger().info("Done.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    static_camera_publisher = StaticTFCamPublisher()

    rclpy.spin(static_camera_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    static_camera_publisher.destroy_node()
    rclpy.shutdown()
