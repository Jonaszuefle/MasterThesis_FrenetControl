from geometry_msgs.msg import WrenchStamped
import tf2_ros
from tf2_ros import TransformException
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import numpy as np

class ForceTransformation(Node):
    """Transforms the human force meassured at tool0 into the base frame of the KUKA"""

    def __init__(self):
        super().__init__('force_transform_tool0_to_base')

        self.create_subscription(WrenchStamped, '/fts_me_messsysteme_broadcaster_topic_based/external_wrench', self.human_force_callback, 1)
        self.base_frame_force_pub = self.create_publisher(WrenchStamped, '/human_force_base_frame', 1)

        # tf2 listener and buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.rot_kuka = R.from_quat([0, 0, 0, 1]) # default

        self.timer = self.create_timer(0.05, self.lookup_transform)

    def lookup_transform(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'iiwa_base',        # target frame
                'tool0',       # source frame
                Time()          # "latest available"
            )

            q = t.transform.rotation
            quat = [q.x, q.y, q.z, q.w]

            self.rot_kuka = R.from_quat(quat)

        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed: {ex}")

    def human_force_callback(self, msg):
        force_tool0 = np.array([msg.wrench.force.x,
                  msg.wrench.force.y,
                  msg.wrench.force.z])

        force_base_frame = self.rot_kuka.apply(force_tool0)

        out = WrenchStamped()
        out.header = msg.header
        out.header.frame_id = 'iiwa_base'
        out.wrench.force.x, out.wrench.force.y, out.wrench.force.z = force_base_frame
        out.wrench.torque = msg.wrench.torque

        self.base_frame_force_pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = ForceTransformation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()