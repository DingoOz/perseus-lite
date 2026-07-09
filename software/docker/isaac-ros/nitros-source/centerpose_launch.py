"""
Stage 8 perception-level proof-of-life: image -> DnnImageEncoderNode ->
TensorRTNode -> CenterPoseDecoderNode, same chain shape as Stage 5/5-followup/
7, this time for monocular 3D object pose estimation.

Unlike every DNN stage before this one, isaac_ros_centerpose's own test
fixture (centerpose_shoe.onnx) is a REAL TRAINED model (not random
weights) -- NVIDIA ships a matching ground_truth.json for its test image,
so centerpose_check.py can validate actual detection count/location, not
just message structure. See centerpose_check.py.

All parameters below (image dims, image_mean/stddev, the 7 named
tensor_rt outputs, output_field_size, cuboid_scaling_factor,
score_threshold, object_name) are copied from NVIDIA's own
test_centerpose_pol.py, not chosen by us -- they match how
centerpose_shoe.onnx itself was trained/exported.
"""
import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

THIS_DIR = os.path.dirname(os.path.abspath(__file__))

OUTPUT_TENSOR_NAMES = [
    'bboxes', 'scores', 'kps', 'clses',
    'obj_scale', 'kps_displacement_mean', 'kps_heatmap_mean',
]


def generate_launch_description():
    model_file_path = os.path.join(THIS_DIR, 'models', 'centerpose_shoe.onnx')

    encoder_node = ComposableNode(
        name='dnn_image_encoder',
        package='isaac_ros_dnn_image_encoder',
        plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
        namespace='',
        parameters=[{
            'input_image_width': 600,
            'input_image_height': 800,
            'network_image_width': 512,
            'network_image_height': 512,
            'input_encoding': 'rgb8',
            'image_mean': [0.408, 0.447, 0.47],
            'image_stddev': [0.289, 0.274, 0.278],
            'enable_padding': True,
            'tensor_name': 'input_tensor',
        }],
        remappings=[('tensors', 'tensor_pub')],
    )

    tensor_rt_node = ComposableNode(
        name='centerpose_inference',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        namespace='',
        parameters=[{
            'model_file_path': model_file_path,
            'engine_file_path': '/tmp/centerpose_shoe.plan',
            'input_tensor_names': ['input_tensor'],
            'input_binding_names': ['input'],
            'input_tensor_formats': ['nitros_tensor_list_nchw_rgb_f32'],
            'output_tensor_names': OUTPUT_TENSOR_NAMES,
            'output_binding_names': OUTPUT_TENSOR_NAMES,
            'output_tensor_formats': ['nitros_tensor_list_nhwc_rgb_f32'],
            'verbose': False,
            'force_engine_update': False,
            'max_workspace_size': 512 * 1024 * 1024,
        }],
    )

    centerpose_decoder_node = ComposableNode(
        name='centerpose_decoder_node',
        package='isaac_ros_centerpose',
        plugin='nvidia::isaac_ros::centerpose::CenterPoseDecoderNode',
        namespace='',
        parameters=[{
            'output_field_size': [128, 128],
            'cuboid_scaling_factor': 1.0,
            'score_threshold': 0.3,
            'object_name': 'shoe',
        }],
    )

    container = ComposableNodeContainer(
        package='rclcpp_components',
        name='centerpose_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            encoder_node, tensor_rt_node, centerpose_decoder_node],
        output='both',
        arguments=['--ros-args', '--log-level', 'info'],
    )
    return LaunchDescription([container])
