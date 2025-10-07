#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from ur_msgs.msg import IOStates
from ur3e_mrc_msgs.msg import PositionUR3e
from ur3e_mrc_msgs.msg import GripperInput

class UR3eMRC_topics(Node):
    """Node for publishing modified joint states and io states topics."""

    def __init__(self):
        super().__init__("ur3e_mrc_enme480_js_io")

        # subscribing to the UR3e joint states topic from the UR driver
        self.sub_js_ = self.create_subscription(JointState, "joint_states", self.js_callback, 10)
        self.sub_js_  # prevent unused variable warning
        self.get_logger().info(f"Subscribed to joint states")

        # # subscribing to the UR3e IO topic from the UR driver
        # self.sub_io_ = self.create_subscription(IOStates, "/io_and_status_controller/io_states", self.io_callback, 10)
        # self.sub_io_  # prevent unused variable warning
        # self.get_logger().info(f"Subscribed to hardware io states")

        # publishing modified joint states values
        self.pub_pos_ = self.create_publisher(PositionUR3e, "ur3e/position", 10)

        # # publishing gripper related topics
        # self.pub_grasp_ = self.create_publisher(Bool, "gripper/grasping", 10)
        # self.pub_grip_ = self.create_publisher(GripperInput, "ur3e/gripper_input", 10)

    # callback for joint states subscriber
    def js_callback(self, msg):
        pos_msg = PositionUR3e()

        pos_msg.position = [msg.position[5] - math.pi/2, msg.position[0], msg.position[1], msg.position[2] + math.pi/2, msg.position[3], msg.position[4]]
        pos_msg.is_ready = True

        self.pub_pos_.publish(pos_msg)

    # # callback for IO subscriber
    # def io_callback(self, msg):
    #     grasp_msg = Bool()
    #     grip_msg = GripperInput()

    #     grasp_msg.data = msg.digital_in_states[0].state

    #     if (msg.digital_in_states[0].state):
    #         grip_msg.dig_in = 1
    #     else:
    #         grip_msg.dig_in = 0
    #     grip_msg.a_in0 = msg.analog_in_states[0].state
    #     grip_msg.a_in1 = 0.0

    #     self.pub_grasp_.publish(grasp_msg)
    #     self.pub_grip_.publish(grip_msg)

def main(args=None):
    rclpy.init(args=args)

    ur3e_mrc_enme480_js_io = UR3eMRC_topics()
    try:
        rclpy.spin(ur3e_mrc_enme480_js_io)
    except RuntimeError as err:
        ur3e_mrc_enme480_js_io.get_logger().error(str(err))
    except SystemExit:
        rclpy.logging.get_logger("ur3e_mrc_enme480_js_io").info(f"JS IO node exited")

    rclpy.shutdown()


if __name__ == "__main__":
    main()