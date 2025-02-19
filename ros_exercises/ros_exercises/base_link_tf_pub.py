import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import numpy as np
import tf_transformations as tft


def makeTransform(translation, rotation):
    """
    Takes a translation in 3d and a rotation in quaternion, outputs a transform.
    """
    transformation = tft.quaternion_matrix(rotation)
    transformation[0:3, 3] = translation
    return transformation


def unmakeTransform(transform):
    """
    Takes a transform, outputs a 3d translation and a quaternion rotation
    """
    rotation = tft.quaternion_from_matrix(transform)
    translation = transform[0:3, 3]
    return translation, rotation


class BaseLinkTFPublisher(Node):
    def __init__(self):
        super().__init__("base_link_tf_pub")

        self.br = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.left_to_base = np.eye(4)
        self.left_to_base[1, 3] = -0.05

        # Create a timer to update the dynamic transform at 10 Hz.
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Get the current time.
        now = self.get_clock().now()
        try:
            trans = self.tf_buffer.lookup_transform("odom", "left_cam", now)
        except Exception as e:
            self.get_logger().info(f"Left camera transform failed:: {e}")
            return

        left_trans = [
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z,
        ]
        left_rot = [
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w,
        ]
        odom_to_left = makeTransform(left_trans, left_rot)
        odom_to_base = np.dot(odom_to_left, self.left_to_base)
        translation, rotation = unmakeTransform(odom_to_base)

        # Create the TransformStamped message for odom -> base_link_2.
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link_2"
        msg.transform.translation.x = float(translation[0])
        msg.transform.translation.y = float(translation[1])
        msg.transform.translation.z = float(translation[2])
        msg.transform.rotation.x = float(rotation[0])
        msg.transform.rotation.y = float(rotation[1])
        msg.transform.rotation.z = float(rotation[2])
        msg.transform.rotation.w = float(rotation[3])

        # Broadcast the composed transform.
        self.br.sendTransform(msg)


def main(args=None):
    rclpy.init(args=args)

    base_link_publisher = BaseLinkTFPublisher()

    rclpy.spin(base_link_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    base_link_publisher.destroy_node()
    rclpy.shutdown()
