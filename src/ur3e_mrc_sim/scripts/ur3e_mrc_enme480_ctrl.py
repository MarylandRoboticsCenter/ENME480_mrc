#!/usr/bin/env python3

# import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# from std_msgs.msg import Bool
# from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
# from ur_msgs.msg import IOStates
# from ur3e_mrc_msgs.msg import PositionUR3e
# from ur3e_mrc_msgs.msg import GripperInput

class UR3eMRC_ctrl(Node):
    """Node for for controlling an UR3e arm using JS topics."""

    def __init__(self):
        super().__init__("ur3e_mrc_enme480_ctrl")

        controller_name = "scaled_joint_trajectory_controller/follow_joint_trajectory"

        self._action_client = ActionClient(self, FollowJointTrajectory, controller_name)
        self.get_logger().info(f"Waiting for action server on {controller_name}")
        self._action_client.wait_for_server()


        # # subscribing to the UR3e IO topic from the UR driver
        # self.sub_io_ = self.create_subscription(IOStates, "/io_and_status_controller/io_states", self.io_callback, 10)
        # self.sub_io_  # prevent unused variable warning
        # self.get_logger().info("Subscribed to hardware io states")

        # # publishing modified joint states values
        # self.pub_pos_ = self.create_publisher(PositionUR3e, "ur3e/position", 10)

        # # publishing gripper related topics
        # self.pub_grasp_ = self.create_publisher(Bool, "gripper/grasping", 10)
        # self.pub_grip_ = self.create_publisher(GripperInput, "ur3e/gripper_input", 10)

    # callback for joint states subscriber
    def js_callback(self, msg):
        pos_msg = PositionUR3e()

        pos_msg.position = [msg.position[5] - math.pi/2, msg.position[0], msg.position[1], msg.position[2] + math.pi/2, msg.position[3], msg.position[4]]
        pos_msg.is_ready = True

        self.pub_pos_.publish(pos_msg)



def main(args=None):
    rclpy.init(args=args)

    ur3e_mrc_enme480_ctrl = UR3eMRC_ctrl()
    try:
        rclpy.spin(ur3e_mrc_enme480_ctrl)
    except RuntimeError as err:
        ur3e_mrc_enme480_ctrl.get_logger().error(str(err))
    except SystemExit:
        rclpy.logging.get_logger("ur3e_mrc_enme480_ctrl").info("UR3e ctrl node exited")

    rclpy.shutdown()


if __name__ == "__main__":
    main()