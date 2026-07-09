"""
Stage 11: deep stereo disparity estimation (ESS network), genuinely new
CUDA territory compared to Stages 5-8 -- not a classify/detect/segment
tensor_rt chain, but a *pair* of image pipelines (left/right: format
convert -> resize -> normalize -> to-tensor -> planar -> reshape) synced
by a TensorPairSyncNode, fed into one two-input/two-output TensorRTNode,
decoded by DNNStereoDecoderNode (which does its own CUDA disparity
filtering -- filter_disparity.cu.cpp, a real .cu kernel, not just a
TensorRT wrapper like Stage 5's tensor_rt node).

This is a straight port of NVIDIA's own isaac_ros_ess_test.py graph
(isaac_ros_dnn_stereo_depth/isaac_ros_ess/test/isaac_ros_ess_test.py) --
node types, parameters, and remappings copied verbatim, not designed by
us. Their own test publishes all-zero synthetic images (not a real
photo pair) and only validates output shape/encoding/calibration-derived
fields, not disparity content -- same "structural proof, not accuracy"
scope as Stage 5/5-follow-up/7 (untrained/dummy weights). See
ess_stereo_check.py.
"""
import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

THIS_DIR = os.path.dirname(os.path.abspath(__file__))

IMAGE_WIDTH = 1920
IMAGE_HEIGHT = 1080
MODEL_WIDTH = 960
MODEL_HEIGHT = 576
NUM_CHANNELS = 3


def _side_nodes(side):
    """Build the format->resize->normalize->tensor->planar->reshape chain
    for one side ('left' or 'right'), matching NVIDIA's test exactly."""
    return [
        ComposableNode(
            name=f'{side}_format_node',
            package='isaac_ros_image_proc',
            plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
            namespace='/ess',
            parameters=[{
                'image_width': IMAGE_WIDTH,
                'image_height': IMAGE_HEIGHT,
                'encoding_desired': 'rgb8',
            }],
            remappings=[
                ('image_raw', f'{side}/image_rect'),
                ('image', f'{side}/image_rgb'),
            ],
        ),
        ComposableNode(
            name=f'{side}_resize_node',
            package='isaac_ros_image_proc',
            plugin='nvidia::isaac_ros::image_proc::ResizeNode',
            namespace='/ess',
            parameters=[{
                'output_width': MODEL_WIDTH,
                'output_height': MODEL_HEIGHT,
                'keep_aspect_ratio': False,
            }],
            remappings=[
                ('image', f'{side}/image_rgb'),
                ('camera_info', f'{side}/camera_info_rect'),
                ('resize/image', f'{side}/image_resize'),
                ('resize/camera_info', f'{side}/camera_info_resize'),
            ],
        ),
        ComposableNode(
            name=f'{side}_normalize_node',
            package='isaac_ros_image_proc',
            plugin='nvidia::isaac_ros::image_proc::ImageNormalizeNode',
            namespace='/ess',
            parameters=[{
                'mean': [127.5, 127.5, 127.5],
                'stddev': [127.5, 127.5, 127.5],
            }],
            remappings=[
                ('image', f'{side}/image_resize'),
                ('normalized_image', f'{side}/image_normalize'),
            ],
        ),
        ComposableNode(
            name=f'{side}_tensor_node',
            package='isaac_ros_tensor_proc',
            plugin='nvidia::isaac_ros::dnn_inference::ImageToTensorNode',
            namespace='/ess',
            parameters=[{'scale': False, 'tensor_name': f'{side}_image'}],
            remappings=[
                ('image', f'{side}/image_normalize'),
                ('tensor', f'{side}/tensor'),
            ],
        ),
        ComposableNode(
            name=f'{side}_planar_node',
            package='isaac_ros_tensor_proc',
            plugin='nvidia::isaac_ros::dnn_inference::InterleavedToPlanarNode',
            namespace='/ess',
            parameters=[{
                'input_tensor_shape': [MODEL_HEIGHT, MODEL_WIDTH, NUM_CHANNELS],
                'output_tensor_name': f'{side}_image',
            }],
            remappings=[
                ('interleaved_tensor', f'{side}/tensor'),
                ('planar_tensor', f'{side}/tensor_planar'),
            ],
        ),
        ComposableNode(
            name=f'{side}_reshape_node',
            package='isaac_ros_tensor_proc',
            plugin='nvidia::isaac_ros::dnn_inference::ReshapeNode',
            namespace='/ess',
            parameters=[{
                'output_tensor_name': f'{side}_image',
                'input_tensor_shape': [NUM_CHANNELS, MODEL_HEIGHT, MODEL_WIDTH],
                'output_tensor_shape': [1, NUM_CHANNELS, MODEL_HEIGHT, MODEL_WIDTH],
            }],
            remappings=[
                ('tensor', f'{side}/tensor_planar'),
                ('reshaped_tensor', f'{side}/tensor_reshape'),
            ],
        ),
    ]


def generate_launch_description():
    model_file_path = os.path.join(THIS_DIR, 'models', 'ess_dummy_model.onnx')

    tensor_pair_sync_node = ComposableNode(
        name='tensor_pair_sync_node',
        package='isaac_ros_tensor_proc',
        plugin='nvidia::isaac_ros::dnn_inference::TensorPairSyncNode',
        namespace='/ess',
        parameters=[{
            'input_tensor1_name': 'left_image',
            'input_tensor2_name': 'right_image',
            'output_tensor1_name': 'input_left',
            'output_tensor2_name': 'input_right',
        }],
        remappings=[
            ('tensor1', 'left/tensor_reshape'),
            ('tensor2', 'right/tensor_reshape'),
        ],
    )

    tensor_rt_node = ComposableNode(
        name='tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        namespace='/ess',
        parameters=[{
            'model_file_path': model_file_path,
            'engine_file_path': '/tmp/ess_dummy.plan',
            'input_tensor_names': ['input_left', 'input_right'],
            'input_binding_names': ['input_left', 'input_right'],
            'output_tensor_names': ['output_left', 'output_conf'],
            'output_binding_names': ['output_left', 'output_conf'],
            'verbose': False,
            'force_engine_update': False,
        }],
    )

    decoder_node = ComposableNode(
        name='dnn_stereo_decoder',
        package='isaac_ros_dnn_stereo_decoder',
        plugin='nvidia::isaac_ros::dnn_stereo_depth::DNNStereoDecoderNode',
        namespace='/ess',
        parameters=[{
            'disparity_tensor_name': 'output_left',
            'confidence_tensor_name': 'output_conf',
        }],
        remappings=[('right/camera_info', 'right/camera_info_resize')],
    )

    container = ComposableNodeContainer(
        package='rclcpp_components',
        name='ess_stereo_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=(
            _side_nodes('left') + _side_nodes('right')
            + [tensor_pair_sync_node, tensor_rt_node, decoder_node]
        ),
        output='both',
        arguments=['--ros-args', '--log-level', 'info'],
    )
    return LaunchDescription([container])
