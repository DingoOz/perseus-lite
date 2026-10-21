# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Mission orchestrator -- owns the segment state machine and the SLAM/Nav2 stacks.

Reads the mission descriptor (mission.yaml), tracks which segment the viewer
is in, and publishes mission state on /mission/state for the web UI.

It also *owns* the two stacks the student repairs, each as its own
``ros2 launch`` process group (see ``process_group.ManagedProcess``):

* ``slam`` -- ``slam_toolbox online_async_launch.py`` (configures and
  activates the lifecycle node), started with the broken params. When
  Challenge 1 passes it is stopped and restarted with the reference params.
* ``nav2`` -- ``autonomy perseus_nav_bringup.launch.py``, started after
  ``nav2_start_delay_s`` with the broken params. When the nav2 params
  checker reports the student's file fixed (Challenge 2's params step) it
  is restarted with the reference params; Challenge 2 then passes when the
  rover reaches the leak under Nav2.

Owning the stacks (instead of having mission_zero.launch.py start them)
means a restart only ever signals process groups this node started -- no
``pkill`` of unrelated slam_toolbox instances, no orphaned node processes.
Kill-and-relaunch is used rather than a lifecycle/param reconfigure because
slam_toolbox's resolution cannot be changed on a running map.

Challenge events are accepted in mission order only (``MissionProgress``).
Everything runs on the executor thread: process supervision is a steady-
clock timer that polls, so no thread ever calls into rclpy.
"""

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .checkers.config_validation import load_yaml_file
from .mission_logic import (  # noqa: F401 (find_segment_index re-exported)
    find_segment_index,
    merge_unlocked_badges,
    MissionProgress,
)
from .process_group import EXITED, ManagedProcess, RUNNING, STOPPING

DEFAULT_CHECKER_TOPICS = (
    '/mission/challenge_1/passed',
    '/mission/challenge_2/passed',
)


def slam_launch_argv(params_file: str, use_sim_time: bool) -> list:
    """Return the command that starts (and activates) slam_toolbox."""
    return [
        'ros2',
        'launch',
        'slam_toolbox',
        'online_async_launch.py',
        f'slam_params_file:={params_file}',
        f'use_sim_time:={str(use_sim_time).lower()}',
        'autostart:=true',
    ]


def nav2_launch_argv(params_file: str, use_sim_time: bool) -> list:
    """Return the command that starts the Nav2 stack."""
    return [
        'ros2',
        'launch',
        'autonomy',
        'perseus_nav_bringup.launch.py',
        f'params_file:={params_file}',
        f'use_sim_time:={str(use_sim_time).lower()}',
        'use_composition:=False',
    ]


class Stack:
    """A named, restartable ``ros2 launch`` process group."""

    def __init__(self, name: str, argv_factory, logger):
        self.name = name
        self.proc = ManagedProcess(name)
        self._argv_factory = argv_factory
        self._logger = logger
        self._pending_params = None
        self._stopping = False
        self.params_file = None

    def start(self, params_file: str) -> None:
        """Start the stack, or restart it if it is already running."""
        if self.proc.state in (RUNNING, STOPPING):
            self._pending_params = params_file
            if not self._stopping:
                self._logger.info(f'Stopping {self.name} to restart it.')
                self._stopping = True
                self.proc.request_stop()
            return
        self._launch(params_file)

    def _launch(self, params_file: str) -> None:
        argv = self._argv_factory(params_file)
        self._logger.info(f'Starting {self.name}: {" ".join(argv)}')
        try:
            self.proc.start(argv)
        except OSError as e:
            self._logger.error(f'Failed to start {self.name}: {e}')
            return
        self.params_file = params_file

    def poll(self) -> None:
        """Supervise the group: finish restarts, report unexpected exits."""
        before = self.proc.state
        state = self.proc.poll()
        if state != EXITED or before == EXITED:
            return
        if self._stopping:
            self._stopping = False
            self._logger.info(f'{self.name} stopped.')
        else:
            self._logger.error(
                f'{self.name} exited unexpectedly (code {self.proc.returncode}).'
            )
        if self._pending_params is not None:
            params, self._pending_params = self._pending_params, None
            self._launch(params)

    def describe(self) -> dict:
        """Return a JSON-friendly summary for /mission/state."""
        return {'state': self.proc.state, 'params_file': self.params_file}

    def shutdown(self) -> None:
        """Stop the group synchronously (node shutdown)."""
        self._pending_params = None
        self.proc.stop_blocking(sigint_timeout=3.0, sigterm_timeout=1.5)


class MissionOrchestrator(Node):
    """Walk the mission segments and drive the SLAM/Nav2 restarts."""

    def __init__(self):
        super().__init__('mission_orchestrator')
        # use_sim_time is declared by rclpy itself; set it from the launch
        # file's parameters, never re-declare it here.
        self.declare_parameter('mission_yaml', '')
        self.declare_parameter('slam_params_file', '')
        self.declare_parameter('nav2_params_file', '')
        self.declare_parameter('slam_reference_params', '')
        self.declare_parameter('nav2_reference_params', '')
        self.declare_parameter('manage_slam', True)
        self.declare_parameter('manage_nav2', True)
        self.declare_parameter('nav2_start_delay_s', 12.0)

        mission_path = Path(self.get_parameter('mission_yaml').value)
        doc, error = load_yaml_file(mission_path)
        if error or not isinstance(doc, dict):
            self.get_logger().error(
                f'Mission descriptor unusable at {mission_path} '
                f'({error or "not a mapping"}); orchestrator idle.'
            )
            doc = None
        self.mission = doc
        segments = doc.get('segments', []) if doc else []
        self.progress = MissionProgress(segments if isinstance(segments, list) else [])
        if doc:
            self.get_logger().info(
                f'Loaded mission {doc.get("id")!r} '
                f'({len(self.progress.segments)} segments, '
                f'{len(self.progress.done)} challenges).'
            )

        mission_dir = mission_path.parent
        if self.mission is None:
            mission_dir = (
                Path(get_package_share_directory('perseus_lite_missions'))
                / 'missions'
                / 'm00-cold-open'
            )
        configs = (self.mission or {}).get('configs') or {}
        self.slam_reference = self._resolve(
            'slam_reference_params', mission_dir, configs.get('slam_reference')
        )
        self.nav2_reference = self._resolve(
            'nav2_reference_params', mission_dir, configs.get('nav2_reference')
        )

        self.use_sim_time = bool(self.get_parameter('use_sim_time').value)
        self.stacks = {}
        if self.get_parameter('manage_slam').value:
            self.stacks['slam'] = Stack(
                'slam',
                lambda p: slam_launch_argv(p, self.use_sim_time),
                self.get_logger(),
            )
        if self.get_parameter('manage_nav2').value:
            self.stacks['nav2'] = Stack(
                'nav2',
                lambda p: nav2_launch_argv(p, self.use_sim_time),
                self.get_logger(),
            )

        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.badge_pub = self.create_publisher(String, '/mission/badge', 10)

        for challenge in range(len(self.progress.done)):
            seg = self.progress.challenge_segment(challenge)
            topic = (
                seg.get('checker_topic')
                or DEFAULT_CHECKER_TOPICS[
                    min(challenge, len(DEFAULT_CHECKER_TOPICS) - 1)
                ]
            )
            self.create_subscription(Bool, topic, self._make_passed_cb(challenge), 10)
            params_topic = seg.get('params_checker_topic')
            if params_topic:
                self.create_subscription(
                    Bool, params_topic, self._make_params_cb(challenge), 10
                )

        # Process supervision runs on a steady clock so it works before
        # /clock arrives and while the simulation is paused.
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.5, self._supervise, clock=steady)
        self.create_timer(1.0, self._publish_state)

        if 'slam' in self.stacks:
            self.stacks['slam'].start(self.get_parameter('slam_params_file').value)
        self._nav2_start_timer = None
        if 'nav2' in self.stacks:
            delay = max(float(self.get_parameter('nav2_start_delay_s').value), 0.0)
            self._nav2_start_timer = self.create_timer(
                delay or 0.01, self._start_nav2, clock=steady
            )

    def _resolve(self, param: str, mission_dir: Path, relative) -> str:
        value = self.get_parameter(param).value
        if value:
            return value
        if relative:
            return str(mission_dir / relative)
        return ''

    def _start_nav2(self):
        # One-shot: created on the executor thread, cancelled on first fire.
        self._nav2_start_timer.cancel()
        self.destroy_timer(self._nav2_start_timer)
        self._nav2_start_timer = None
        if 'nav2' in self.stacks and self.stacks['nav2'].params_file is None:
            self.stacks['nav2'].start(self.get_parameter('nav2_params_file').value)

    def _supervise(self):
        for stack in self.stacks.values():
            stack.poll()

    def _make_passed_cb(self, challenge: int):
        def cb(msg: Bool):
            if msg.data:
                self._on_challenge_passed(challenge)

        return cb

    def _make_params_cb(self, challenge: int):
        def cb(msg: Bool):
            if msg.data:
                self._on_params_fixed(challenge)

        return cb

    def _restart_stack_for(self, challenge: int):
        seg = self.progress.challenge_segment(challenge)
        name = seg.get('restart_stack')
        if not name:
            return
        stack = self.stacks.get(name)
        if stack is None:
            self.get_logger().warning(
                f'Challenge {challenge + 1} wants to restart {name!r}, '
                'which this orchestrator does not manage.'
            )
            return
        reference = {'slam': self.slam_reference, 'nav2': self.nav2_reference}.get(
            name, ''
        )
        if not reference or not Path(reference).is_file():
            self.get_logger().error(
                f'Reference params for {name} not found at {reference!r}; '
                f'not restarting {name}.'
            )
            return
        # If Nav2's delayed first start has not fired yet this starts it
        # straight away with the reference params; _start_nav2 then no-ops.
        self.get_logger().info(f'Restarting {name} with reference params {reference}')
        stack.start(reference)

    def _on_params_fixed(self, challenge: int):
        decision = self.progress.on_params_fixed(challenge)
        if not decision.accepted:
            self.get_logger().debug(f'params-fixed ignored: {decision.reason}')
            return
        self.get_logger().info(f'Challenge {challenge + 1} config fixed.')
        seg = self.progress.challenge_segment(challenge)
        if seg.get('params_checker_topic'):
            self._restart_stack_for(challenge)

    def _on_challenge_passed(self, challenge: int):
        if self.progress.done[challenge]:
            return  # checkers republish their latched result every tick
        decision = self.progress.on_challenge_passed(challenge)
        if not decision.accepted:
            self.get_logger().info(
                f'Challenge {challenge + 1} report ignored: {decision.reason}',
                throttle_duration_sec=15.0,
            )
            return
        seg = self.progress.challenge_segment(challenge)
        self.get_logger().info(f'Challenge {challenge + 1} passed: {seg.get("title")}')
        if not seg.get('params_checker_topic'):
            self._restart_stack_for(challenge)
        current = self.progress.current_segment or {}
        self.get_logger().info(
            f'Advanced to segment {current.get("id")}: {current.get("title")}'
        )
        badge = seg.get('badge')
        if badge:
            self._emit_badge(badge)
            self._persist_badges([badge])
        if self.progress.complete:
            self.get_logger().info('All challenges passed -- mission complete.')

    def _publish_state(self):
        seg = self.progress.current_segment
        if seg is None:
            return
        done = self.progress.done
        msg = String()
        msg.data = json.dumps(
            {
                'segment_idx': self.progress.current_idx,
                'segment_id': seg.get('id'),
                'segment_kind': seg.get('kind'),
                'segment_title': seg.get('title'),
                'challenges_done': done,
                'challenges_params_fixed': self.progress.params_fixed,
                'challenge_1_done': len(done) > 0 and done[0],
                'challenge_2_done': len(done) > 1 and done[1],
                'stacks': {n: s.describe() for n, s in self.stacks.items()},
            }
        )
        self.state_pub.publish(msg)

    def _emit_badge(self, badge_id: str):
        for badge in (self.mission or {}).get('badges') or []:
            if isinstance(badge, dict) and badge.get('id') == badge_id:
                msg = String()
                msg.data = json.dumps(badge)
                self.badge_pub.publish(msg)
                self.get_logger().info(f'Badge unlocked: {badge.get("title")}')
                return

    def _persist_badges(self, badge_ids: list):
        badge_file = Path.home() / '.config' / 'rover-robotics' / 'badges.json'
        try:
            badge_file.parent.mkdir(parents=True, exist_ok=True)
            existing = {}
            if badge_file.is_file():
                try:
                    existing = json.loads(badge_file.read_text())
                except (json.JSONDecodeError, UnicodeDecodeError):
                    existing = {}
            existing = merge_unlocked_badges(existing, badge_ids)
            badge_file.write_text(json.dumps(existing, indent=2) + '\n')
        except OSError as e:
            self.get_logger().error(f'Could not persist badges to {badge_file}: {e}')
            return
        self.get_logger().info(f'Badges persisted to {badge_file}')

    def shutdown(self):
        """Stop every stack this node started (whole process groups)."""
        for stack in self.stacks.values():
            stack.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MissionOrchestrator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
