#!/usr/bin/env python3
"""
Compare Nav2 planners on one goal without driving the robot.

Calls planner_server's ComputePathToPose once per planner (from the robot's
current pose) and prints path length, detour ratio, planning time and the
decoded error code. It needs only planner_server (and TF), not bt_navigator,
so it is safe to run while the robot is parked:

    ros2 run autonomy plan_probe.py 3.0 1.5                      # all loaded planners
    ros2 run autonomy plan_probe.py 3.0 1.5 --planners ThetaStar Smac2D
    ros2 run autonomy plan_probe.py 3.0 1.5 --frame odom

A detour ratio well above ~1.5 on open ground means the planner is routing
around something: check /terrain/global_grid and the costmaps for a false
obstacle before blaming the planner.
"""

import argparse
import math
import sys
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# nav2_msgs/action/ComputePathToPose error codes (Jazzy).
ERROR_NAMES = {
    0: "NONE",
    200: "UNKNOWN",
    201: "INVALID_PLANNER",
    202: "TF_ERROR",
    203: "START_OUTSIDE_MAP",
    204: "GOAL_OUTSIDE_MAP",
    205: "START_OCCUPIED",
    206: "GOAL_OCCUPIED",
    207: "TIMEOUT",
    208: "NO_VALID_PATH",
}


def path_length(path):
    """Return the length of a nav_msgs/Path in metres."""
    poses = [p.pose.position for p in path.poses]
    return sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(poses, poses[1:]))


def main():
    """Parse arguments, query each planner, print one line per planner."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("x", type=float, help="goal x")
    parser.add_argument("y", type=float, help="goal y")
    parser.add_argument("--yaw", type=float, default=0.0, help="goal yaw, rad")
    parser.add_argument("--frame", default="map", help="goal frame")
    parser.add_argument(
        "--planners",
        nargs="+",
        default=["GridBased", "ThetaStar", "Smac2D"],
        help="planner_ids to try (as in planner_server planner_plugins)",
    )
    parser.add_argument("--timeout", type=float, default=10.0)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = Node("plan_probe")
    client = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
    if not client.wait_for_server(timeout_sec=5.0):
        print("compute_path_to_pose action server not available", file=sys.stderr)
        return 1

    print(f"goal ({args.x:.2f}, {args.y:.2f}) in {args.frame}")
    for planner in args.planners:
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = args.frame
        goal.goal.pose.position.x = args.x
        goal.goal.pose.position.y = args.y
        goal.goal.pose.orientation.z = math.sin(args.yaw / 2.0)
        goal.goal.pose.orientation.w = math.cos(args.yaw / 2.0)
        goal.planner_id = planner
        goal.use_start = False  # plan from the robot's current pose

        started = time.monotonic()
        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send, timeout_sec=args.timeout)
        handle = send.result()
        if handle is None or not handle.accepted:
            print(f"  {planner:12s} goal rejected")
            continue
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.timeout)
        elapsed = time.monotonic() - started
        response = result_future.result()
        if response is None:
            print(f"  {planner:12s} no result within {args.timeout:.0f} s")
            continue
        result = response.result
        code = ERROR_NAMES.get(result.error_code, str(result.error_code))
        if response.status != GoalStatus.STATUS_SUCCEEDED or not result.path.poses:
            print(f"  {planner:12s} FAILED {code} after {elapsed:.2f} s")
            continue
        length = path_length(result.path)
        start = result.path.poses[0].pose.position
        straight = math.hypot(args.x - start.x, args.y - start.y)
        detour = length / straight if straight > 1e-6 else float("nan")
        print(
            f"  {planner:12s} length {length:5.2f} m  detour x{detour:4.2f}  "
            f"{len(result.path.poses):4d} poses  "
            f"planner {result.planning_time.sec + result.planning_time.nanosec * 1e-9:.3f} s"
        )

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
