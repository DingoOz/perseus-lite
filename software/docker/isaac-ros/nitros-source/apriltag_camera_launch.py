"""
Stage 4 live-camera AprilTag test: usb_cam -> rectify -> apriltag, using our
own camera_c920_params.yaml / camera_c920_info.yaml (see those files for why
-- NVIDIA's own isaac_ros_apriltag_usb_cam.launch.py + usb_cam_params.yaml
has a resolution/calibration mismatch: image_raw defaults to 640x480 but
their bundled camera_info.yaml is calibrated for 1280x720).

This chains isaac_ros_image_proc::RectifyNode in front of AprilTagNode, so
detections are on lens-corrected frames. Earlier testing found rectify ->
apriltag on live frames reproducibly crashed ("terminate called after
throwing an instance of 'nvcv::Exception': ... The tensor handle is null.")
within ~1s of apriltag finishing load, and this launch file skipped
RectifyNode entirely as a workaround. After a reboot, the identical graph
ran clean for 33+ minutes of live frames with no crash -- see
isaac-ros-nitros-source-build.md's "Follow-up after a reboot" section for
the full soak-test writeup and the (still open) hypothesis that the crash
depends on stale CUDA/driver state from a prior crash in the same boot
session, not purely a code-level race. If this crashes again, the
rectify-skipping workaround is one-line to restore: drop rectify_node from
composable_node_descriptions and remap apriltag_node's `image` to
`image_raw` instead of `image_rect` (see git history for the previous
version of this file).
"""
import os

from launch import LaunchDescription
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

    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        namespace='',
        parameters=[{'size': 0.22, 'max_tags': 64, 'tile_size': 4}],
        remappings=[
            ('image', 'image_rect'),
            ('camera_info', 'camera_info_rect'),
        ],
    )

    container = ComposableNodeContainer(
        package='rclcpp_components',
        name='apriltag_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[usb_cam_node, rectify_node, apriltag_node],
        output='both',
        arguments=['--ros-args', '--log-level', 'info'],
    )
    return LaunchDescription([container])
