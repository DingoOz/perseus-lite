"""
Stage 7 perception-level proof-of-life: image -> DnnImageEncoderNode ->
TensorRTNode -> UNetDecoderNode, same chain shape as Stage 5's
dnn_classify_launch.py and the Stage 5 follow-up's yolov8_launch.py, this
time for semantic segmentation.

Uses NVIDIA's own model.dummy.onnx test fixture from isaac_ros_unet's own
test suite (test/dummy_model/model.dummy.onnx) -- random weights, so mask
class assignments are not meaningful; this only proves the full
encode->infer->decode chain runs and produces structurally valid raw +
colorized segmentation masks. See unet_check.py.

network_image_width/height, input/output binding names, and the 20-class
color palette below are copied from NVIDIA's own
isaac_ros_unet_pol_test.py, not chosen by us -- they match how
model.dummy.onnx itself was generated/exported.
"""
import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

THIS_DIR = os.path.dirname(os.path.abspath(__file__))

NETWORK_IMAGE_WIDTH = 960
NETWORK_IMAGE_HEIGHT = 544


def generate_random_color_palette(num_classes):
    import numpy as np
    np.random.seed(0)
    palette = []
    for _ in range(num_classes):
        r = np.random.randint(0, 256)
        g = np.random.randint(0, 256)
        b = np.random.randint(0, 256)
        palette.append(int(r) << 16 | int(g) << 8 | int(b))
    return palette


def generate_launch_description():
    model_file_path = os.path.join(THIS_DIR, 'models', 'model.dummy.onnx')

    encoder_node = ComposableNode(
        name='dnn_image_encoder',
        package='isaac_ros_dnn_image_encoder',
        plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
        namespace='',
        parameters=[{
            'input_image_width': 1200,
            'input_image_height': 632,
            'network_image_width': NETWORK_IMAGE_WIDTH,
            'network_image_height': NETWORK_IMAGE_HEIGHT,
            'input_encoding': 'rgb8',
            'enable_padding': True,
            'tensor_name': 'input_tensor',
        }],
        remappings=[('tensors', 'tensor_pub')],
    )

    tensor_rt_node = ComposableNode(
        name='tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        namespace='',
        parameters=[{
            'model_file_path': model_file_path,
            'engine_file_path': '/tmp/unet_dummy.plan',
            'input_tensor_names': ['input_tensor'],
            'input_binding_names': ['input_1'],
            'output_tensor_names': ['output_tensor'],
            'output_binding_names': ['softmax_1'],
            'verbose': False,
            'force_engine_update': False,
        }],
    )

    unet_decoder_node = ComposableNode(
        name='unet_decoder',
        package='isaac_ros_unet',
        plugin='nvidia::isaac_ros::unet::UNetDecoderNode',
        namespace='',
        parameters=[{
            'color_segmentation_mask_encoding': 'rgb8',
            'color_palette': generate_random_color_palette(20),
            'mask_width': NETWORK_IMAGE_WIDTH,
            'mask_height': NETWORK_IMAGE_HEIGHT,
        }],
    )

    container = ComposableNodeContainer(
        package='rclcpp_components',
        name='unet_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            encoder_node, tensor_rt_node, unet_decoder_node],
        output='both',
        arguments=['--ros-args', '--log-level', 'info'],
    )
    return LaunchDescription([container])
