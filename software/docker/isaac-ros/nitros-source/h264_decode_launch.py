"""
Stage 9 revised scope: EncoderNode failed to open /dev/v4l2-nvenc --
this Orin Nano SKU has no hardware video encoder block at all (only
/dev/v4l2-nvdec exists; confirmed via `ls /dev`, and NVIDIA's own
encoder_v4l2_impl.cpp hardcodes that exact device path). This is a real
hardware limitation (Orin Nano lacks NVENC; only Orin NX/AGX have it),
not a JetPack7/build gap -- see isaac-ros-nitros-source-build.md's
"Stage 9" section.

DecoderNode alone (hardware NVDEC, which this SoC does have) is still
worth proving: this launch feeds NVIDIA's own pre-encoded
compressed.h264 test fixture directly, bypassing EncoderNode entirely.
See h264_decode_check.py.
"""
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    decoder_node = ComposableNode(
        name='decoder',
        package='isaac_ros_h264_decoder',
        plugin='nvidia::isaac_ros::h264_decoder::DecoderNode',
        namespace='',
        parameters=[{
            'output_width': 1920,
            'output_height': 1200,
        }],
    )

    container = ComposableNodeContainer(
        package='rclcpp_components',
        name='h264_decode_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[decoder_node],
        output='both',
        arguments=['--ros-args', '--log-level', 'info'],
    )
    return LaunchDescription([container])
