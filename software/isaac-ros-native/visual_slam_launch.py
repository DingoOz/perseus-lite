"""
Stage 6 decision-gate + proof-of-life: does isaac_ros_visual_slam (cuVSLAM)
load and track against a real RGBD recording on Orin/JetPack7?

Unlike GXF core, cuVSLAM ships a genuine native aarch64_jetpack70 binary
(isaac_ros_nitros/isaac_ros_nitros/lib/cuvslam/lib_aarch64_jetpack70/
libcuvslam.so) -- isaac_ros_nitros's own CMakeLists.txt selects it purely
by CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64", no sbsa-vs-jetson fallback
needed the way GXF's core binaries required.

Plays back NVIDIA's own isaac_ros_visual_slam RGBD test fixture
(test_cases/rosbags/rgbd_static/rosbag2_rs455_rgbd.mcap -- a real
RealSense 455 recording, 6s/92 frames) rather than a live camera; this
robot has no depth/stereo camera, only the monocular C920, and cuVSLAM's
tracking_mode only supports Multicamera(stereo)/VIO(stereo+IMU)/RGBD, no
monocular-only mode -- see isaac-ros-nitros-source-build.md for why this
capability isn't deployable on the current robot hardware as-is.

vslam_parameters below mirror NVIDIA's own
isaac_ros_visual_slam/test/helpers.py:run_cuvslam_rgbd_from_bag (the same
config their isaac_ros_visual_slam_pol_rgbd_cam.py test uses), not chosen
by us. See visual_slam_check.py for the companion odometry check.
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

BAG_PATH = (
    'isaac_ros_visual_slam/isaac_ros_visual_slam/test/test_cases/'
    'rosbags/rgbd_static/rosbag2_rs455_rgbd.mcap'
)


def generate_launch_description():
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        namespace='',
        parameters=[{
            'tracking_mode': 2,  # RGBD mode
            'depth_scale_factor': 1000.0,
            'rectified_images': False,
            'image_jitter_threshold_ms': 30.00,
            'sync_matching_threshold_ms': 10.0,
            'base_frame': 'camera_link',
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            'enable_ground_constraint_in_odometry': False,
            'enable_ground_constraint_in_slam': False,
            'enable_localization_n_mapping': True,
            'min_num_images': 1,
            'num_cameras': 1,
            'depth_camera_id': 0,
            'camera_optical_frames': ['camera_color_optical_frame'],
        }],
        remappings=[
            ('visual_slam/image_0', 'camera/color/image_raw'),
            ('visual_slam/camera_info_0', 'camera/color/camera_info'),
            ('visual_slam/depth_0', 'camera/aligned_depth_to_color/image_raw'),
        ],
    )

    container = ComposableNodeContainer(
        package='rclcpp_components',
        name='visual_slam_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[visual_slam_node],
        output='both',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', BAG_PATH, '--clock'],
        output='both',
    )

    return LaunchDescription([container, rosbag_play])
