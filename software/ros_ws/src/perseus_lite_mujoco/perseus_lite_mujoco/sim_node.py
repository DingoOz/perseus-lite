"""
perseus_lite_mujoco: MuJoCo simulation of Perseus Lite, bridged to ROS 2.

The robot stack runs exactly as on hardware or in Gazebo, except that ros2_control uses
mock_components/GenericSystem, which reports each wheel at its commanded velocity (as
the ST3215 servos closely do). This node:

- reads the wheels' commanded velocities from /joint_states and drives MuJoCo's wheel
  velocity actuators with them;
- steps MuJoCo in real time and publishes /clock;
- publishes the robot's sensors: /scan (C1 lidar, by ray casting, never hitting the
  robot itself), /imu/data (gyro + accelerometer, no orientation, like the real IMU),
  and the OAK-D Lite's /oak/stereo/image_raw (32FC1 depth) and /oak/right_rect/image_raw
  (RGB) with camera_info (offscreen rendering);
- publishes /mujoco/ground_truth (nav_msgs/Odometry in the world frame) for validation.

TF is not published here: robot_state_publisher, the EKF and SLAM own it, as everywhere.
"""

import math
import os
import subprocess
import threading
import time


def main():
    """Entry point. MUJOCO_GL must be chosen before mujoco is imported."""
    import argparse
    import sys

    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--gui', action='store_true')
    known, _ = parser.parse_known_args(sys.argv[1:])
    # Offscreen rendering: EGL needs no display; the interactive viewer needs GLFW.
    os.environ.setdefault('MUJOCO_GL', 'glfw' if known.gui else 'egl')

    import rclpy

    rclpy.init(args=sys.argv)
    node = MujocoSim(gui=known.gui)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.try_shutdown()


class MujocoSim:
    """See the module docstring."""

    def __init__(self, gui):
        """Build the model and the ROS interfaces."""
        import mujoco
        import numpy as np
        from ament_index_python.packages import get_package_share_directory
        from geometry_msgs.msg import TransformStamped  # noqa: F401  (import check)
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data, QoSProfile
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan

        from perseus_lite_mujoco import model_builder as mb

        self.mujoco = mujoco
        self.np = np
        self.msgs = {
            'Clock': Clock,
            'Odometry': Odometry,
            'CameraInfo': CameraInfo,
            'Image': Image,
            'Imu': Imu,
            'LaserScan': LaserScan,
        }
        self.gui = gui
        self.node = Node('mujoco_sim')
        n = self.node
        p = n.declare_parameter
        robot_description = p('robot_description', '').value
        world_file = p('world_file', '').value
        spawn = (
            p('initial_pose_x', 0.0).value,
            p('initial_pose_y', 0.0).value,
            p('initial_pose_z', 0.1).value,
            p('initial_pose_yaw', 0.0).value,
        )
        self.real_time_factor = p('real_time_factor', 1.0).value
        self.clock_rate_hz = p('clock_rate_hz', 250.0).value
        self.imu_rate_hz = p('imu_rate_hz', 100.0).value
        self.scan_rate_hz = p('scan_rate_hz', 10.0).value
        self.camera_rate_hz = p('camera_rate_hz', 10.0).value
        self.camera_enabled = p('camera', True).value
        self.lidar_min, self.lidar_max = (
            p('lidar_min_range', 0.3).value,
            p('lidar_max_range', 12.0).value,
        )
        self.lidar_samples = int(p('lidar_samples', 360).value)
        self.gyro_noise = p('gyro_noise', 2e-4).value
        self.accel_noise = p('accel_noise', 1.7e-2).value
        self.depth_noise = p('depth_noise', 0.007).value
        params = mb.SimParams(
            wheel_friction=tuple(p('wheel_friction', [0.6, 0.005, 0.0001]).value),
            world_friction=tuple(p('world_friction', [0.6, 0.005, 0.0001]).value),
            impratio=p('impratio', 1.0).value,
            wheel_kv=p('wheel_kv', 5.0).value,
            wheel_max_torque=p('wheel_max_torque', 3.0).value,
        )
        self.params = params

        if not robot_description:
            raise RuntimeError('robot_description parameter is empty')
        robot = mb.build_robot(robot_description)
        models_dir = os.path.join(
            get_package_share_directory('perseus_lite_simulation'), 'models'
        )

        def resolve(uri):
            if not uri.startswith('model://'):
                return None
            return os.path.join(models_dir, uri.replace('model://', '', 1))

        geoms, lights, notes = (
            mb.build_world(world_file, resolve) if world_file else ([], [], [])
        )
        if not world_file:
            geoms = [
                mb.WorldGeom(
                    'ground', 'box', (50, 50, 0.05), mb.transform((0, 0, -0.05))
                )
            ]
        for note in robot.skipped + notes:
            n.get_logger().info(f'model: {note}')
        xml = mb.build_mjcf(robot, geoms, lights, spawn, params)
        self.model = mujoco.MjModel.from_xml_string(xml)
        # MuJoCo's clip planes scale with the model extent; a 40 m ground box put the
        # near plane ~0.5 m out, so a tag or wall closer than that vanished from the
        # camera. Pin them in metres instead (near 2 cm, far 60 m).
        extent = max(self.model.stat.extent, 1e-6)
        self.model.vis.map.znear = 0.02 / extent
        self.model.vis.map.zfar = 60.0 / extent
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        n.get_logger().info(
            f'MuJoCo {mujoco.__version__}: {self.model.nbody} bodies, {self.model.ngeom} geoms, '
            f'robot {robot.base.mass + sum(w.mass for w in robot.wheels):.1f} kg, '
            f'world {os.path.basename(world_file) or "flat ground"}, spawn {spawn}'
        )

        # Wheel actuators by joint name; commanded velocities arrive on /joint_states.
        self.actuator_of = {}
        for i in range(self.model.nu):
            joint_id = self.model.actuator_trnid[i, 0]
            self.actuator_of[self.model.joint(joint_id).name] = i
        self.lock = threading.Lock()
        self.wheel_command = {}
        n.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)

        self.clock_pub = n.create_publisher(Clock, '/clock', 10)
        self.imu_pub = n.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.scan_pub = n.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.truth_pub = n.create_publisher(Odometry, '/mujoco/ground_truth', 10)
        if self.camera_enabled:
            reliable = QoSProfile(depth=2)
            self.depth_pub = n.create_publisher(
                Image, '/oak/stereo/image_raw', reliable
            )
            self.depth_info_pub = n.create_publisher(
                CameraInfo, '/oak/stereo/camera_info', reliable
            )
            self.rgb_pub = n.create_publisher(
                Image, '/oak/right_rect/image_raw', reliable
            )
            self.rgb_info_pub = n.create_publisher(
                CameraInfo, '/oak/right_rect/camera_info', reliable
            )
            self.renderer = mujoco.Renderer(
                self.model, params.camera_height, params.camera_width
            )
            self.depth_renderer = mujoco.Renderer(
                self.model, params.camera_height, params.camera_width
            )
            self.depth_renderer.enable_depth_rendering()

        site = self.model.site
        self.lidar_site = site(params.lidar_link).id
        self.imu_site = site(params.imu_link).id
        self.gyro_adr = self.model.sensor('imu_gyro').adr[0]
        self.accel_adr = self.model.sensor('imu_accel').adr[0]
        self.gyro_slice = slice(self.gyro_adr, self.gyro_adr + 3)
        self.accel_slice = slice(self.accel_adr, self.accel_adr + 3)
        self.base_body = self.model.body(robot.base.name).id
        # Lidar sees only static world geometry (group 0): never the robot itself.
        self.ray_groups = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        angles = np.linspace(-math.pi, math.pi, self.lidar_samples, endpoint=False)
        self.scan_angles = angles
        self.scan_local = np.stack(
            [np.cos(angles), np.sin(angles), np.zeros_like(angles)], axis=1
        )
        self.rng = np.random.default_rng(0)
        self.stop = threading.Event()
        self.viewer = None

    # ---- inputs ---------------------------------------------------------------------

    def on_joint_states(self, msg):
        """Take the commanded wheel velocities (mock hardware reports command as state)."""
        with self.lock:
            for name, velocity in zip(msg.name, msg.velocity):
                if name in self.actuator_of:
                    self.wheel_command[name] = velocity

    # ---- outputs --------------------------------------------------------------------

    def stamp(self):
        """Return the current sim time as a builtin_interfaces/Time."""
        from builtin_interfaces.msg import Time

        t = self.data.time
        return Time(sec=int(t), nanosec=int((t - int(t)) * 1e9))

    def publish_clock(self):
        """Publish /clock."""
        msg = self.msgs['Clock']()
        msg.clock = self.stamp()
        self.clock_pub.publish(msg)

    def publish_imu(self):
        """Publish /imu/data from the gyro and accelerometer sensors (site frame)."""
        msg = self.msgs['Imu']()
        msg.header.stamp = self.stamp()
        msg.header.frame_id = self.params.imu_link
        gyro = self.data.sensordata[self.gyro_slice] + self.rng.normal(
            0, self.gyro_noise, 3
        )
        accel = self.data.sensordata[self.accel_slice] + self.rng.normal(
            0, self.accel_noise, 3
        )
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
            float(v) for v in gyro
        )
        (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ) = (float(v) for v in accel)
        msg.orientation_covariance[0] = -1.0  # no orientation, as the real I2C IMU
        for i in (0, 4, 8):
            msg.angular_velocity_covariance[i] = self.gyro_noise**2
            msg.linear_acceleration_covariance[i] = self.accel_noise**2
        self.imu_pub.publish(msg)

    def publish_scan(self):
        """Publish /scan by casting rays in the lidar frame's plane."""
        mujoco, np = self.mujoco, self.np
        origin = self.data.site_xpos[self.lidar_site].copy()
        rotation = self.data.site_xmat[self.lidar_site].reshape(3, 3)
        directions = (self.scan_local @ rotation.T).astype(np.float64).ravel()
        n = self.lidar_samples
        geom_ids = np.zeros(n, dtype=np.int32)
        distances = np.zeros(n, dtype=np.float64)
        mujoco.mj_multiRay(
            self.model,
            self.data,
            origin,
            directions,
            self.ray_groups,
            1,
            -1,
            geom_ids,
            distances,
            None,
            n,
            self.lidar_max,
        )
        ranges = np.where(
            (distances < 0) | (distances > self.lidar_max), np.inf, distances
        )
        ranges = np.where(ranges < self.lidar_min, np.inf, ranges)
        msg = self.msgs['LaserScan']()
        msg.header.stamp = self.stamp()
        msg.header.frame_id = self.params.lidar_link
        msg.angle_min = float(self.scan_angles[0])
        msg.angle_max = float(self.scan_angles[-1])
        msg.angle_increment = float(2 * math.pi / n)
        msg.scan_time = 1.0 / self.scan_rate_hz
        msg.range_min = float(self.lidar_min)
        msg.range_max = float(self.lidar_max)
        msg.ranges = [float(r) for r in ranges]
        self.scan_pub.publish(msg)

    def camera_info(self):
        """Build the camera_info of the simulated OAK-D (square pixels, centred)."""
        p = self.params
        fx = (p.camera_width / 2.0) / math.tan(p.camera_hfov / 2.0)
        cx, cy = (p.camera_width - 1) / 2.0, (p.camera_height - 1) / 2.0
        msg = self.msgs['CameraInfo']()
        msg.header.stamp = self.stamp()
        msg.header.frame_id = p.camera_link
        msg.width, msg.height = p.camera_width, p.camera_height
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0] * 5
        msg.k = [fx, 0.0, cx, 0.0, fx, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fx, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def publish_camera(self):
        """Render and publish the depth and colour images."""
        np = self.np
        info = self.camera_info()
        self.depth_renderer.update_scene(self.data, camera='oak')
        depth = self.depth_renderer.render().astype(np.float32)
        depth = depth + self.rng.normal(0.0, self.depth_noise, depth.shape).astype(
            np.float32
        )
        far = depth > 10.0
        depth[far] = np.nan
        image = self.msgs['Image']()
        image.header = info.header
        image.height, image.width = depth.shape
        image.encoding = '32FC1'
        image.step = image.width * 4
        image.data = depth.tobytes()
        self.depth_pub.publish(image)
        self.depth_info_pub.publish(info)
        self.renderer.update_scene(self.data, camera='oak')
        rgb = self.renderer.render()
        colour = self.msgs['Image']()
        colour.header = info.header
        colour.height, colour.width = rgb.shape[:2]
        colour.encoding = 'rgb8'
        colour.step = colour.width * 3
        colour.data = rgb.tobytes()
        self.rgb_pub.publish(colour)
        self.rgb_info_pub.publish(info)

    def publish_truth(self):
        """Publish the base body's true pose and velocity in the world frame."""
        d = self.data
        msg = self.msgs['Odometry']()
        msg.header.stamp = self.stamp()
        msg.header.frame_id = 'world'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = (
            float(v) for v in d.qpos[0:3]
        )
        w, x, y, z = (float(v) for v in d.qpos[3:7])
        msg.pose.pose.orientation.w, msg.pose.pose.orientation.x = w, x
        msg.pose.pose.orientation.y, msg.pose.pose.orientation.z = y, z
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = (
            float(v) for v in d.qvel[0:3]
        )
        (
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ) = (float(v) for v in d.qvel[3:6])
        self.truth_pub.publish(msg)

    # ---- main loop ------------------------------------------------------------------

    def run(self):
        """Step physics in real time on this thread; ROS callbacks on another."""
        import rclpy

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self.node)
        spinner = threading.Thread(target=executor.spin, daemon=True)
        spinner.start()
        if self.gui:
            import mujoco.viewer

            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        mujoco = self.mujoco
        periods = {
            'clock': 1.0 / self.clock_rate_hz,
            'imu': 1.0 / self.imu_rate_hz,
            'scan': 1.0 / self.scan_rate_hz,
            'camera': 1.0 / self.camera_rate_hz,
            'truth': 0.05,
        }
        due = dict.fromkeys(periods, 0.0)
        start_wall = time.monotonic()
        while not self.stop.is_set() and rclpy.ok():
            with self.lock:
                for name, velocity in self.wheel_command.items():
                    self.data.ctrl[self.actuator_of[name]] = velocity
            mujoco.mj_step(self.model, self.data)
            t = self.data.time
            if t >= due['clock']:
                self.publish_clock()
                due['clock'] = t + periods['clock']
            if t >= due['imu']:
                self.publish_imu()
                due['imu'] = t + periods['imu']
            if t >= due['scan']:
                self.publish_scan()
                due['scan'] = t + periods['scan']
            if self.camera_enabled and t >= due['camera']:
                self.publish_camera()
                due['camera'] = t + periods['camera']
            if t >= due['truth']:
                self.publish_truth()
                if self.viewer is not None:
                    if not self.viewer.is_running():
                        break
                    self.viewer.sync()
                due['truth'] = t + periods['truth']
            # Real-time pacing (rendering steps may run behind; never sleep negative).
            if self.real_time_factor > 0:
                ahead = t / self.real_time_factor - (time.monotonic() - start_wall)
                if ahead > 0.002:
                    time.sleep(ahead)

    def shutdown(self):
        """Release the renderers and the node."""
        self.stop.set()
        if self.viewer is not None:
            self.viewer.close()
        for name in ('renderer', 'depth_renderer'):
            renderer = getattr(self, name, None)
            if renderer is not None:
                renderer.close()
        self.node.destroy_node()


def xacro_robot_description(xacro_file, args=()):
    """Run xacro (used by the calibration tool, which has no launch file)."""
    return subprocess.run(
        ['xacro', xacro_file, *args], capture_output=True, text=True, check=True
    ).stdout


if __name__ == '__main__':
    main()
