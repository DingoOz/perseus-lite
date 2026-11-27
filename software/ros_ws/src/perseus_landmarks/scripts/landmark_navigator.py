#!/usr/bin/env python3
"""
landmark_navigator: search for AprilTags, and drive to named destinations.

Action servers
  /landmarks/search             perseus_interfaces/action/SearchForLandmarks
      Rotate in place in steps (Nav2 Spin, collision-checked), stopping to look after
      each; if nothing is found and use_search_points is set, drive to each search
      point (Nav2 NavigateToPose) and look again. Optionally calibrate on success.
  /landmarks/go_to_destination  perseus_interfaces/action/GoToDestination
      Localise first if needed (search + calibrate), drive to the destination in the
      site frame, and if it has a tag: find it, let the localiser re-anchor on it, and
      drive the last bit again with the corrected estimate.

The robot only moves through Nav2 (Spin, NavigateToPose), so every motion goes through
the costmaps, the collision monitor and twist_mux, where the gamepad always wins.
"""

import collections
import math
import threading
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from perseus_interfaces.action import GoToDestination, SearchForLandmarks
from perseus_interfaces.msg import LandmarkLocalisation, TagDetectionArray
from perseus_interfaces.srv import CalibrateFromLandmarks
from perseus_landmarks.landmark_map import load_landmark_map
from perseus_landmarks.search_plan import rotation_steps, start_relative_to_map
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros


class Cancelled(Exception):
    """Raised inside a running action when its goal is cancelled."""


def yaw_to_quaternion(yaw):
    """Return (z, w) of a planar rotation quaternion."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class LandmarkNavigator(Node):
    """See the module docstring."""

    def __init__(self):
        """Declare parameters and create the action servers and clients."""
        super().__init__("landmark_navigator")
        self.landmarks = load_landmark_map(
            self.declare_parameter("landmarks_file", "").value
        )
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.robot_frame = self.declare_parameter("robot_frame", "base_link").value
        self.step_rad = math.radians(self.declare_parameter("step_deg", 45.0).value)
        self.dwell_s = self.declare_parameter("dwell_s", 1.5).value
        self.max_rotation_rad = math.radians(
            self.declare_parameter("max_rotation_deg", 360.0).value
        )
        self.seen_timeout_s = self.declare_parameter("seen_timeout_s", 1.0).value
        # The localiser's hard gates: a sighting it would reject must not end a search
        # (keep these equal to landmark_localiser's max_range_m, max_view_angle_deg and
        # max_reprojection_px).
        self.min_sightings = self.declare_parameter("min_sightings", 3).value
        self.max_range_m = self.declare_parameter("max_range_m", 4.0).value
        self.max_view_angle_rad = math.radians(
            self.declare_parameter("max_view_angle_deg", 60.0).value
        )
        self.max_reprojection_px = self.declare_parameter(
            "max_reprojection_px", 2.0
        ).value
        self.calibrate_timeout_s = self.declare_parameter(
            "calibrate_timeout_s", 6.0
        ).value
        self.refine_tolerance_m = self.declare_parameter(
            "refine_tolerance_m", 0.15
        ).value
        self.settle_s = self.declare_parameter("settle_s", 2.0).value
        self.nav_timeout_s = self.declare_parameter("nav_timeout_s", 180.0).value
        self.auto_localise = self.declare_parameter("auto_localise", True).value
        search_on_start = self.declare_parameter("search_on_start", False).value
        startup_delay_s = self.declare_parameter("startup_delay_s", 15.0).value

        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        # (time, id) of recent usable sightings; "found" needs min_sightings of them.
        self.sightings = collections.deque(maxlen=1000)
        self.calibrated = False
        self.busy = threading.Lock()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(
            TagDetectionArray,
            "/landmarks/detections",
            self.on_detections,
            10,
            callback_group=self.group,
        )
        self.create_subscription(
            LandmarkLocalisation,
            "/landmarks/localisation",
            self.on_localisation,
            10,
            callback_group=self.group,
        )
        self.spin_client = ActionClient(self, Spin, "spin", callback_group=self.group)
        self.nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=self.group
        )
        self.calibrate_client = self.create_client(
            CalibrateFromLandmarks,
            "/landmark_localiser/calibrate",
            callback_group=self.group,
        )
        self.search_server = ActionServer(
            self,
            SearchForLandmarks,
            "/landmarks/search",
            execute_callback=self.execute_search,
            goal_callback=self.accept_if_idle,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.group,
        )
        self.destination_server = ActionServer(
            self,
            GoToDestination,
            "/landmarks/go_to_destination",
            execute_callback=self.execute_destination,
            goal_callback=self.accept_if_idle,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.group,
        )
        if search_on_start:
            self.startup_timer = self.create_timer(
                startup_delay_s, self.startup_search, callback_group=self.group
            )
        self.get_logger().info(
            f"landmark_navigator: {len(self.landmarks.tag_ids)} tags, "
            f"{len(self.landmarks.destinations)} destinations, "
            f"{len(self.landmarks.search_points)} search points"
        )

    # ---- inputs ---------------------------------------------------------------------

    def on_detections(self, msg):
        """Remember when each known tag was last seen usably (passing the localiser's gates)."""
        now = time.monotonic()
        with self.lock:
            for detection in msg.detections:
                if (
                    detection.id in self.landmarks.tag_ids
                    and detection.distance_m <= self.max_range_m
                    and detection.view_angle_rad <= self.max_view_angle_rad
                    and detection.reprojection_error_px <= self.max_reprojection_px
                ):
                    self.sightings.append((now, detection.id))

    def on_localisation(self, msg):
        """Track whether the localiser has a site -> map fix."""
        self.calibrated = msg.calibrated

    def seen(self, targets):
        """
        Return the target ids seen steadily: min_sightings within seen_timeout_s.

        One sighting is not enough: a tag at the edge of the view flickers in and out,
        and a search that stops on it leaves the localiser too few sightings to fix.
        """
        now = time.monotonic()
        with self.lock:
            counts = collections.Counter(
                tag
                for stamp, tag in self.sightings
                if now - stamp <= self.seen_timeout_s
            )
        return sorted(i for i in targets if counts[i] >= self.min_sightings)

    def accept_if_idle(self, _goal):
        """Run one landmark action at a time: they all move the robot."""
        return GoalResponse.REJECT if self.busy.locked() else GoalResponse.ACCEPT

    # ---- blocking helpers (run in action threads; the executor keeps spinning) ------

    def wait(self, future, timeout_s, goal_handle=None):
        """Wait for a future; raise Cancelled if the goal is cancelled meanwhile."""
        deadline = time.monotonic() + timeout_s
        while not future.done():
            if goal_handle is not None and goal_handle.is_cancel_requested:
                raise Cancelled()
            if time.monotonic() > deadline:
                return None
            time.sleep(0.05)
        return future.result()

    def run_action(self, client, goal, timeout_s, goal_handle, interrupt=None):
        """
        Send an action goal and wait for its result; cancel it if we are cancelled.

        With interrupt (a callable), the goal is also cancelled as soon as interrupt()
        returns true, and "interrupted" is returned.
        """
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f"{client._action_name} action server not available"
            )
            return False
        sent = self.wait(client.send_goal_async(goal), 10.0, goal_handle)
        if sent is None or not sent.accepted:
            return False
        result_future = sent.get_result_async()
        if interrupt is not None:
            deadline = time.monotonic() + timeout_s
            while not result_future.done() and time.monotonic() < deadline:
                if goal_handle is not None and goal_handle.is_cancel_requested:
                    sent.cancel_goal_async()
                    raise Cancelled()
                if interrupt():
                    self.wait(sent.cancel_goal_async(), 5.0)
                    return "interrupted"
                time.sleep(0.02)
        try:
            result = self.wait(result_future, timeout_s, goal_handle)
        except Cancelled:
            sent.cancel_goal_async()
            raise
        if result is None:
            sent.cancel_goal_async()
            return False
        return result.status == GoalStatus.STATUS_SUCCEEDED

    def dwell(self, targets, goal_handle):
        """Look for dwell_s; return the targets seen."""
        deadline = time.monotonic() + self.dwell_s
        while time.monotonic() < deadline:
            if goal_handle is not None and goal_handle.is_cancel_requested:
                raise Cancelled()
            found = self.seen(targets)
            if found:
                return found
            time.sleep(0.1)
        return self.seen(targets)

    def robot_pose(self, frame):
        """Return the robot's (x, y, yaw) in a frame, or None."""
        try:
            t = self.tf_buffer.lookup_transform(frame, self.robot_frame, Time())
        except tf2_ros.TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        return t.transform.translation.x, t.transform.translation.y, yaw

    def navigate(self, frame, x, y, yaw, goal_handle):
        """Drive to a pose with Nav2."""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = frame
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = yaw_to_quaternion(
            yaw
        )
        return self.run_action(self.nav_client, goal, self.nav_timeout_s, goal_handle)

    def glimpsed(self, targets, since):
        """Return whether any target was sighted (once is enough) after `since`."""
        with self.lock:
            return any(
                stamp >= since and tag in targets for stamp, tag in self.sightings
            )

    def spin(self, angle, goal_handle, watch=None):
        """
        Turn in place by angle (Nav2 Spin, collision-checked); return (ok, heading).

        With watch (target ids), stop as soon as one is glimpsed and return the robot's
        map heading at that moment: Spin's slow outer loop overshoots by tens of
        degrees, enough to carry a tag straight out of the 73 deg view.
        """
        goal = Spin.Goal()
        goal.target_yaw = float(angle)
        goal.time_allowance = Duration(sec=int(10 + 8 * abs(angle)))
        start = time.monotonic()
        seen_at = []

        def interrupt():
            if watch and self.glimpsed(watch, start):
                pose = self.robot_pose(self.map_frame)
                seen_at.append(pose[2] if pose else None)
                return True
            return False

        result = self.run_action(
            self.spin_client,
            goal,
            15 + 8 * abs(angle),
            goal_handle,
            interrupt if watch else None,
        )
        if result == "interrupted":
            return True, seen_at[0] if seen_at else None
        return bool(result), None

    def calibrate(self, goal_handle):
        """Ask the localiser for a stationary fix."""
        if not self.calibrate_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("/landmark_localiser/calibrate is not available")
            return False, "localiser not running"
        request = CalibrateFromLandmarks.Request()
        request.timeout_s = float(self.calibrate_timeout_s)
        response = self.wait(
            self.calibrate_client.call_async(request),
            self.calibrate_timeout_s + 3.0,
            goal_handle,
        )
        if response is None:
            return False, "calibrate did not answer"
        return response.success, response.message

    # ---- search ---------------------------------------------------------------------

    def search(
        self,
        targets,
        max_rotation,
        use_search_points,
        goal_handle,
        feedback_cls=None,
        calibrate=False,
    ):
        """
        Look for any of targets; return (ids found, calibrated, message).

        With calibrate, a sighting only ends the search once the localiser has taken a
        fix from it; if the fix fails (too few usable sightings), keep turning.
        """

        def feedback(phase, rotated=0.0, index=-1):
            if goal_handle is not None and feedback_cls is not None:
                message = feedback_cls()
                message.phase = phase
                message.rotated_rad = float(rotated)
                message.search_point_index = index
                goal_handle.publish_feedback(message)

        points = [None]
        if use_search_points:
            points += list(self.landmarks.search_points)
        start = self.robot_pose(self.map_frame)
        for index, point in enumerate(points):
            if point is not None:
                if point.frame == "site":
                    if not self.calibrated:
                        continue
                    frame, (x, y) = self.landmarks.site_frame, (point.x, point.y)
                else:
                    if start is None:
                        continue
                    frame, (x, y) = self.map_frame, start_relative_to_map(point, *start)
                feedback("driving", 0.0, index - 1)
                here = self.robot_pose(frame)
                heading = math.atan2(y - here[1], x - here[0]) if here else 0.0
                if not self.navigate(frame, x, y, heading, goal_handle):
                    self.get_logger().warn(
                        f"search point {index - 1} unreachable; skipping it"
                    )
                    continue
            feedback("looking", 0.0, index - 1)
            rotated = 0.0
            steps = list(rotation_steps(max_rotation, self.step_rad))
            while True:
                found = self.dwell(targets, goal_handle)
                if found:
                    if not calibrate:
                        return found, False, f"found tags {found}"
                    feedback("calibrating", rotated, index - 1)
                    ok, message = self.calibrate(goal_handle)
                    if ok:
                        return found, True, message
                    self.get_logger().warn(
                        f"saw tags {found} but {message}; turning on"
                    )
                if not steps:
                    break
                step = steps.pop(0)
                feedback("rotating", rotated, index - 1)
                ok, seen_heading = self.spin(step, goal_handle, watch=targets)
                if not ok:
                    self.get_logger().warn(
                        "spin refused or failed (obstacle?); looking from here and moving on"
                    )
                    break
                rotated += step
                if seen_heading is not None and not self.dwell(targets, goal_handle):
                    # Glimpsed while turning but the stop overshot it: turn back to the
                    # heading where it was seen, then look again (top of the loop).
                    here = self.robot_pose(self.map_frame)
                    if here is not None:
                        back = (seen_heading - here[2] + math.pi) % (
                            2 * math.pi
                        ) - math.pi
                        if abs(back) > math.radians(3.0):
                            feedback("rotating", rotated, index - 1)
                            self.spin(back, goal_handle)
        return [], False, "no usable target tag found"

    def execute_search(self, goal_handle):
        """Run a SearchForLandmarks goal."""
        with self.busy:
            request = goal_handle.request
            targets = list(request.tag_ids) or list(self.landmarks.tag_ids)
            result = SearchForLandmarks.Result()
            try:
                found, calibrated, message = self.search(
                    targets,
                    request.max_rotation_rad or self.max_rotation_rad,
                    request.use_search_points,
                    goal_handle,
                    SearchForLandmarks.Feedback,
                    calibrate=request.calibrate,
                )
                result.found = bool(found)
                result.tag_ids_seen = found
                result.calibrated = calibrated
                result.message = f"found tags {found}; {message}" if found else message
            except Cancelled:
                goal_handle.canceled()
                result.message = "cancelled"
                return result
            self.get_logger().info(f"search: {result.message}")
            if result.found:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result

    def startup_search(self):
        """One-shot: search for any known tag and calibrate (search_on_start)."""
        self.startup_timer.cancel()
        with self.busy:
            self.get_logger().info(
                "search_on_start: looking for a known tag to calibrate from"
            )
            try:
                found, calibrated, message = self.search(
                    list(self.landmarks.tag_ids),
                    self.max_rotation_rad,
                    False,
                    None,
                    calibrate=True,
                )
            except Cancelled:
                return
            if calibrated:
                self.get_logger().info(f"search_on_start: {message}")
            else:
                self.get_logger().warn(f"search_on_start: {message} after a full turn")

    # ---- destinations ---------------------------------------------------------------

    def execute_destination(self, goal_handle):
        """Run a GoToDestination goal."""
        with self.busy:
            result = GoToDestination.Result()
            result.final_error_m = -1.0
            name = goal_handle.request.destination
            destination = self.landmarks.destinations.get(name)

            def fail(message):
                result.message = message
                self.get_logger().warn(f"go_to_destination {name}: {message}")
                goal_handle.abort()
                return result

            def feedback(phase):
                message = GoToDestination.Feedback()
                message.phase = phase
                goal_handle.publish_feedback(message)

            if destination is None:
                return fail(
                    f"unknown destination (known: {sorted(self.landmarks.destinations)})"
                )
            try:
                if not self.calibrated:
                    if not self.auto_localise:
                        return fail(
                            "not localised in the site frame (run /landmarks/search with calibrate)"
                        )
                    feedback("localising")
                    found, calibrated, message = self.search(
                        list(self.landmarks.tag_ids),
                        self.max_rotation_rad,
                        True,
                        goal_handle,
                        calibrate=True,
                    )
                    if not calibrated:
                        return fail(f"could not localise: {message}")
                site = self.landmarks.site_frame
                # A fix taken a moment ago may not have reached Nav2's TF buffer yet;
                # a goal in an unknown frame is rejected outright.
                deadline = time.monotonic() + 3.0
                while self.robot_pose(site) is None and time.monotonic() < deadline:
                    time.sleep(0.1)
                time.sleep(0.5)
                feedback("navigating")
                if not self.navigate(
                    site, destination.x, destination.y, destination.yaw, goal_handle
                ):
                    return fail("navigation failed")
                if destination.tag is not None:
                    # Re-anchor on the destination's tag, then correct the final pose.
                    feedback("searching")
                    found, _, _ = self.search(
                        [destination.tag], self.max_rotation_rad, False, goal_handle
                    )
                    if found:
                        # Take a stationary fix from the destination's tag. Waiting for
                        # the live estimate is not enough: the drift accumulated on the
                        # way is often a "jump" (> max_jump_m), which the localiser only
                        # accepts after jump_confirm_s, so the error would be measured
                        # against the stale estimate and the refine skipped.
                        ok, message = self.calibrate(goal_handle)
                        if not ok:
                            self.get_logger().warn(
                                f"re-anchoring on tag {destination.tag} failed ({message});"
                                " using the live estimate"
                            )
                            time.sleep(self.settle_s)
                        error = self.error_to(destination)
                        if error is not None and error > self.refine_tolerance_m:
                            feedback("refining")
                            self.navigate(
                                site,
                                destination.x,
                                destination.y,
                                destination.yaw,
                                goal_handle,
                            )
                    else:
                        self.get_logger().warn(
                            f"destination tag {destination.tag} not found; arrived on the estimate alone"
                        )
                result.final_error_m = float(self.error_to(destination) or -1.0)
            except Cancelled:
                goal_handle.canceled()
                result.message = "cancelled"
                return result
            result.success = True
            result.message = f"arrived at {name}"
            self.get_logger().info(
                f"go_to_destination: {result.message} (error {result.final_error_m:.2f} m)"
            )
            goal_handle.succeed()
            return result

    def error_to(self, destination):
        """Return the distance from the robot to a destination by the site estimate."""
        pose = self.robot_pose(self.landmarks.site_frame)
        if pose is None:
            return None
        return math.hypot(pose[0] - destination.x, pose[1] - destination.y)


def main():
    """Run the navigator on a multi-threaded executor (actions block their own thread)."""
    rclpy.init()
    node = LandmarkNavigator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
