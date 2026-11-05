import argparse
import math

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

from panda_bringup.model import settings


def main(args=None):
    parser = argparse.ArgumentParser(description="Execute a timed joint trajectory; no collision planning")
    parser.add_argument("positions", nargs=7, type=float)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--namespace", default="panda")
    parsed = parser.parse_args(args)
    config = settings()
    if not math.isfinite(parsed.duration) or parsed.duration < 1.0:
        parser.error("duration must be finite and at least one second")
    for name, q, low, high in zip(config["joint_names"], parsed.positions, config["lower"], config["upper"]):
        if not math.isfinite(q) or not low <= q <= high:
            parser.error(f"{name}: position must be within [{low}, {high}]")
    rclpy.init()
    node = Node("joint_goal")
    client = ActionClient(node, FollowJointTrajectory,
                          f"/{parsed.namespace.strip('/')}/panda_arm_controller/follow_joint_trajectory")
    try:
        if not client.wait_for_server(timeout_sec=15):
            raise RuntimeError("Trajectory controller is not available")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = config["joint_names"]
        point = JointTrajectoryPoint(positions=parsed.positions, velocities=[0.0] * 7,
                                     accelerations=[0.0] * 7)
        seconds = int(parsed.duration)
        point.time_from_start = Duration(sec=seconds, nanosec=round((parsed.duration - seconds) * 1e9))
        goal.trajectory.points = [point]
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done() or not future.result().accepted:
            raise RuntimeError("Trajectory rejected or request timed out")
        handle = future.result()
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result, timeout_sec=parsed.duration + 10)
        if not result.done():
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=3)
            raise RuntimeError("Trajectory timed out; cancellation requested")
        response = result.result().result
        if result.result().status != GoalStatus.STATUS_SUCCEEDED or response.error_code:
            raise RuntimeError(f"Trajectory failed: {response.error_code}: {response.error_string}")
        print("Target reached")
    finally:
        node.destroy_node()
        rclpy.shutdown()
