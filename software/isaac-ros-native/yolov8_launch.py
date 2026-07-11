"""
Stage 5 perception-level proof-of-life (parallel to Stage 4's AprilTag):
image -> DnnImageEncoderNode -> TensorRTNode -> YoloV8DecoderNode, using
NVIDIA's own dummy_yolov8s.onnx test fixture (real yolov8s architecture/IO
names, random weights -- same fixture and params as NVIDIA's own
isaac_ros_yolov8_decoder_node_pol.py, minus the isaac_ros_test/
IsaacROSBaseTest dependency). Random weights mean detected boxes are
meaningless; this only proves the full encode->infer->decode chain runs
and produces a structurally valid Detection2DArray -- see
yolov8_check.py.

image_mean/image_stddev and the exact input/output tensor+binding names
below are copied from NVIDIA's test, not chosen by us -- they match how
dummy_yolov8s.onnx itself was generated/exported.
"""
import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

THIS_DIR = os.path.dirname(os.path.abspath(__file__))

INPUT_TENSOR_NAMES = ['input_tensor']
INPUT_BINDING_NAMES = ['images']
OUTPUT_TENSOR_NAMES = ['output_tensor']
OUTPUT_BINDING_NAMES = ['output0']


def generate_launch_description():
    model_file_path = os.path.join(THIS_DIR, 'models', 'dummy_yolov8s.onnx')

    encoder_node = ComposableNode(
        name='dnn_image_encoder',
        package='isaac_ros_dnn_image_encoder',
        plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
        namespace='',
        parameters=[{
            'input_image_width': 640,
            'input_image_height': 640,
            'network_image_width': 640,
            'network_image_height': 640,
            'input_encoding': 'rgb8',
            'image_mean': [0.5, 0.6, 0.25],
            'image_stddev': [0.25, 0.8, 0.5],
            'enable_padding': True,
            'tensor_name': INPUT_TENSOR_NAMES[0],
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
            'engine_file_path': '/tmp/dummy_yolov8s.plan',
            'output_binding_names': OUTPUT_BINDING_NAMES,
            'output_tensor_names': OUTPUT_TENSOR_NAMES,
            'input_tensor_names': INPUT_TENSOR_NAMES,
            'input_binding_names': INPUT_BINDING_NAMES,
            'verbose': False,
            'force_engine_update': False,
        }],
    )

    yolov8_decoder_node = ComposableNode(
        name='yolov8_decoder',
        package='isaac_ros_yolov8',
        plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
        namespace='',
        parameters=[{
            'tensor_name': OUTPUT_TENSOR_NAMES[0],
            'confidence_threshold': 0.25,
            'nms_threshold': 0.45,
        }],
    )

    container = ComposableNodeContainer(
        package='rclcpp_components',
        name='yolov8_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            encoder_node, tensor_rt_node, yolov8_decoder_node],
        output='both',
        arguments=['--ros-args', '--log-level', 'info'],
    )
    return LaunchDescription([container])
