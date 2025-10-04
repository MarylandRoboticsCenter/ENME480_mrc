#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class UR3eMRC_js(Node):
    """Client for publishing modified joint states topic."""

    def __init__(self):
        super().__init__("jtc_client")
        self.declare_parameter("controller_name", "scaled_joint_trajectory_controller")
        self.declare_parameter(
            "joints",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )

def main(args=None):
    rclpy.init(args=args)

    jtc_client = JTCClient()
    try:
        rclpy.spin(jtc_client)
    except RuntimeError as err:
        jtc_client.get_logger().error(str(err))
    except SystemExit:
        rclpy.logging.get_logger("jtc_client").info("Done")

    rclpy.shutdown()


if __name__ == "__main__":
    main()