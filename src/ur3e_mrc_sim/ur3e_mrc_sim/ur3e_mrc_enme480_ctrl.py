#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from builtin_interfaces.msg import Duration
# from std_msgs.msg import Bool
# from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTolerance
from action_msgs.msg import GoalStatus
# from ur_msgs.msg import IOStates
from ur3e_mrc_msgs.msg import CommandUR3e
# from ur3e_mrc_msgs.msg import PositionUR3e
# from ur3e_mrc_msgs.msg import GripperInput

N_JOINTS = 6

class UR3eMRC_ctrl(Node):
    """Node for for controlling an UR3e arm using JS topics."""

    def __init__(self):
        super().__init__("ur3e_mrc_enme480_ctrl")

        self.comm_group = MutuallyExclusiveCallbackGroup()

        controller_name = "scaled_joint_trajectory_controller/follow_joint_trajectory"

        self._action_client = ActionClient(self, FollowJointTrajectory, controller_name)
        self.get_logger().info(f"Waiting for action server on {controller_name}")

        init_status_ = False
        init_status_ = self._action_client.wait_for_server(timeout_sec=20.0)
        if (init_status_):
            self.get_logger().info(f"Connected to the scaled_joint_trajectory_action server")
        else:
            raise Exception("Failed to connected to the scaled_joint_trajectory_action server")

        # subscribing to the UR3e command topic that is used to set joint angles
        self.sub_comm_ = self.create_subscription(CommandUR3e, "ur3e/command", self.comm_callback, 10, callback_group=self.comm_group)
        self.sub_comm_  # prevent unused variable warning
        self.get_logger().info(f"Subscribed to ur3e command")

        self.ur3e_isReady = True

        self.goal = JointTrajectory()
        self.goal.trajectory.joint_names.append("shoulder_pan_joint")
        self.goal.trajectory.joint_names.append("shoulder_lift_joint")
        self.goal.trajectory.joint_names.append("elbow_joint")
        self.goal.trajectory.joint_names.append("wrist_1_joint")
        self.goal.trajectory.joint_names.append("wrist_2_joint")
        self.goal.trajectory.joint_names.append("wrist_3_joint")
        
        self.point = JointTrajectoryPoint()
        self.point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.goal.points.append(self.point)

        self._send_goal_future = None
        self._get_result_future = None        


    # callback for joint states command subscriber
    def comm_callback(self, msg):
        self.get_logger().info(f"Got ur3e command")
        if (msg.destination is None) or (len(msg.destination) != N_JOINTS):
            self.get_logger().warn(f"WARNNING: wrong number of values in ur3e command")
            return
        
        self.goal.trajectory.points[0].positions = msg.destination
        self.goal.trajectory.points[0].positions[0] += math.pi / 2
        self.goal.trajectory.points[0].positions[3] -= math.pi / 2
        self.goal.trajectory.points[0].time_from_start = Duration(sec=2, nanosec=0)

        if (self.ur3e_isReady):

            self.ur3e_isReady = False
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = self.goal

            goal.goal_time_tolerance = Duration(sec=0, nanosec=500000000)
            goal.goal_tolerance = [JointTolerance(position=0.01, velocity=0.01, name=self.joints[i]) for i in range(6)]

            self._send_goal_future = self._action_client.send_goal_async(goal)
            self._send_goal_future.add_done_callback(self.goal_response_callback)            
        
        else:
            self.get_logger().info(f"Trajectory controller is not ready yet")


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected :(")
            raise RuntimeError("Goal rejected :(")

        self.get_logger().debug("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f"Done with result: {self.status_to_str(status)}")
        if status == GoalStatus.STATUS_SUCCEEDED:
            time.sleep(1)
            self.ur3e_isReady = True
        else:
            if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                self.get_logger().error(f"Done with result: {self.error_code_to_str(result.error_code)}")
            raise RuntimeError("Executing trajectory failed. " + result.error_string)

    @staticmethod
    def error_code_to_str(error_code):
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            return "SUCCESSFUL"
        if error_code == FollowJointTrajectory.Result.INVALID_GOAL:
            return "INVALID_GOAL"
        if error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
            return "INVALID_JOINTS"
        if error_code == FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP:
            return "OLD_HEADER_TIMESTAMP"
        if error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
            return "PATH_TOLERANCE_VIOLATED"
        if error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
            return "GOAL_TOLERANCE_VIOLATED"

    @staticmethod
    def status_to_str(error_code):
        if error_code == GoalStatus.STATUS_UNKNOWN:
            return "UNKNOWN"
        if error_code == GoalStatus.STATUS_ACCEPTED:
            return "ACCEPTED"
        if error_code == GoalStatus.STATUS_EXECUTING:
            return "EXECUTING"
        if error_code == GoalStatus.STATUS_CANCELING:
            return "CANCELING"
        if error_code == GoalStatus.STATUS_SUCCEEDED:
            return "SUCCEEDED"
        if error_code == GoalStatus.STATUS_CANCELED:
            return "CANCELED"
        if error_code == GoalStatus.STATUS_ABORTED:
            return "ABORTED"


def main(args=None):
    rclpy.init(args=args)

    ur3e_mrc_enme480_ctrl = UR3eMRC_ctrl()

    executor = MultiThreadedExecutor()
    executor.add_node(ur3e_mrc_enme480_ctrl)    
    executor.spin()

    try:
        rclpy.spin(ur3e_mrc_enme480_ctrl)
    except RuntimeError as err:
        ur3e_mrc_enme480_ctrl.get_logger().error(str(err))
    except SystemExit:
        rclpy.logging.get_logger("ur3e_mrc_enme480_ctrl").info("UR3e ctrl node exited")

    rclpy.shutdown()


if __name__ == "__main__":
    main()