import rclpy
import numpy as np

from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf_transformations as tft


class DynamicCamPublisher(Node):

    def __init__(self):
        super().__init__("open_space_publisher")
        self.br = TransformBroadcaster(self)
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.base_to_left = np.eye(4)
        self.base_to_left[1, 3] = 0.05  # +0.05m along y

        self.base_to_right = np.eye(4)
        self.base_to_right[1, 3] = -0.05  # -0.05m along y

    def timer_callback(self):

        now = rclpy.time.Time()

        try:
            # Look up the transform from 'odom' to 'base_link'
            trans = self.tf_buffer.lookup_transform("odom", "base_link", now)
        except Exception as e:
            self.get_logger().info(f"Base link transform failed: {e}")
            return

        base_trans = [
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z,
        ]
        base_rot = [
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w,
        ]

        odom_to_base = makeTransform(base_trans, base_rot)

        odom_to_left = np.dot(odom_to_base, self.base_to_left)

        left_trans, left_rot = unmakeTransform(odom_to_left)
        odom_to_right = np.dot(odom_to_base, self.base_to_right)

        left_to_odom = np.linalg.inv(odom_to_left)
        left_to_right = np.dot(left_to_odom, odom_to_right)
        right_trans, right_rot = unmakeTransform(left_to_right)

        left_msg = TransformStamped()
        left_msg.header.stamp = self.get_clock().now().to_msg()
        left_msg.header.frame_id = "odom"
        left_msg.child_frame_id = "left_cam"
        left_msg.transform.translation.x = float(left_trans[0])
        left_msg.transform.translation.y = float(left_trans[1])
        left_msg.transform.translation.z = float(left_trans[2])
        left_msg.transform.rotation.x = float(left_rot[0])
        left_msg.transform.rotation.y = float(left_rot[1])
        left_msg.transform.rotation.z = float(left_rot[2])
        left_msg.transform.rotation.w = float(left_rot[3])
        self.br.sendTransform(left_msg)

        right_msg = TransformStamped()
        right_msg.header.stamp = self.get_clock().now().to_msg()
        right_msg.header.frame_id = "left_cam"
        right_msg.child_frame_id = "right_cam"
        right_msg.transform.translation.x = float(right_trans[0])
        right_msg.transform.translation.y = float(right_trans[1])
        right_msg.transform.translation.z = float(right_trans[2])
        right_msg.transform.rotation.x = float(right_rot[0])
        right_msg.transform.rotation.y = float(right_rot[1])
        right_msg.transform.rotation.z = float(right_rot[2])
        right_msg.transform.rotation.w = float(right_rot[3])
        self.br.sendTransform(right_msg)


def main(args=None):
    rclpy.init(args=args)

    dynamic_camera_publisher = DynamicCamPublisher()

    rclpy.spin(dynamic_camera_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dynamic_camera_publisher.destroy_node()
    rclpy.shutdown()


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
