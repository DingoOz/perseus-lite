"""
Demo 1 (see plan.md): live GPU-vs-CPU AprilTag detection speed comparison.

usb_cam -> RectifyNode (GPU/VPI) -> image_rect, fanned out to two
independent detectors consuming the identical rectified frames:
  - AprilTagNode (GPU, cuAprilTag via VPI) -- the same node proven stable
    over a 3-minute live soak test, unchanged.
  - demo1_cpu_apriltag_node.py (CPU, pupil-apriltags) -- new for this demo.

demo1_speed_compare_node.py measures rolling FPS + stamp-to-arrival
latency for each stream, draws a stats overlay on both, and publishes a
single side-by-side comparison image plus the two individual streams.

Run: `pixi run -e isaac-nitros demo-apriltag-speed`. View on a laptop on
the same ROS domain -- see README.md's "Live camera test" section for the
ROS_DOMAIN_ID/RMW setup, then either:
  ros2 run rviz2 rviz2 -d demo1_apriltag_speed.rviz
  ros2 run rqt_image_view rqt_image_view /image_comparison
"""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

THIS_DIR = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    usb_cam_params_path = os.path.join(THIS_DIR, 'camera_c920_params.yaml')
    camera_info_path = os.path.join(THIS_DIR, 'camera_c920_info.yaml')

    usb_cam_node = ComposableNode(
        package='usb_cam',
        plugin='usb_cam::UsbCamNode',
        name='usb_cam',
        parameters=[
            usb_cam_params_path,
            {'camera_info_url': f'file://{camera_info_path}'},
        ],
    )

    rectify_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        name='rectify',
        namespace='',
        parameters=[{'output_width': 1280, 'output_height': 720}],
    )

    apriltag_gpu_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        namespace='',
        parameters=[{'size': 0.22, 'max_tags': 64, 'tile_size': 4}],
        remappings=[
            # *_gpu_active, not the raw camera-rate topics -- the pump
            # (demo1_frame_pump_node.py) only publishes here during its
            # GPU phase, so this node only receives frames when it isn't
            # competing with the CPU detector for system resources.
            ('image', 'image_gpu_active'),
            ('camera_info', 'camera_info_gpu_active'),
        ],
    )

    container = ComposableNodeContainer(
        package='rclcpp_components',
        name='apriltag_speed_demo_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[usb_cam_node, rectify_node, apriltag_gpu_node],
        output='both',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    # Plain scripts, not installed ROS packages -- matches this whole project's
    # pattern of ExecuteProcess'd standalone scripts (e.g. `ros2 bag play` in
    # earlier stages) rather than building throwaway ament_python packages.
    frame_pump_node = ExecuteProcess(
        cmd=['python3', os.path.join(THIS_DIR, 'demo1_frame_pump_node.py')],
        output='both',
    )
    apriltag_cpu_node = ExecuteProcess(
        cmd=['python3', os.path.join(THIS_DIR, 'demo1_cpu_apriltag_node.py')],
        output='both',
    )
    speed_compare_node = ExecuteProcess(
        cmd=['python3', os.path.join(THIS_DIR, 'demo1_speed_compare_node.py')],
        output='both',
    )

    return LaunchDescription(
        [container, frame_pump_node, apriltag_cpu_node, speed_compare_node])
