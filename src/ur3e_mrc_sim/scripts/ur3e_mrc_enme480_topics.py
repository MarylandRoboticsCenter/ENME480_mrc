#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from ur3e_mrc_msgs.msg import PositionUR3e

class UR3eMRC_topics(Node):
    """Client for publishing modified joint states and io states topics."""

    def __init__(self):
        super().__init__("ur3e_mrc_enme480_js_io")

        # subscribing to the UR3e joint states topic from the UR driver
        self.sub_js = self.create_subscription(JointState, "joint_states", self.js_callback, 10)
        self.sub_js  # prevent unused variable warning
        self.get_logger().info("Subscribed to joint states")

        # publishing modified joint states values
        self.pub = self.create_publisher(PositionUR3e, 'ur3e/position', 10)

    # callback for joint states subscriber
    def js_callback(self, msg):

        # ur3e_mrc_msgs::msg::PositionUR3e pos_msg;

        # pos_msg.position = {msg.position[5] - boost::math::constants::pi<double>() / 2, msg.position[0], msg.position[1], msg.position[2] + boost::math::constants::pi<double>() / 2, msg.position[3], msg.position[4]};
        # pos_msg.is_ready = true;

        # pos_pub_->publish(pos_msg);

        self.list_1 = msg.data.tolist()
        if (self.list_1[0] == self.list_2[0]):
            self.data_proc()    

def main(args=None):
    rclpy.init(args=args)

    js_io_subs = UR3eMRC_topics()
    try:
        rclpy.spin(js_io_subs)
    except RuntimeError as err:
        js_io_subs.get_logger().error(str(err))
    except SystemExit:
        rclpy.logging.get_logger("js_io_subs").info("JS IO node exited")

    rclpy.shutdown()


if __name__ == "__main__":
    main()