"""
Stage 5 proof-of-life: a static test image -> DnnImageEncoderNode (resize to
224x224, normalize) -> TensorRTNode (mobilenetv2-1.0) chained in one
container, using our own from-source isaac_ros_dnn_image_encoder and
isaac_ros_tensor_rt builds. Not NVIDIA's own encoder+tensor_rt sample launch
(pulls in more machinery than needed for a single-image proof); this is our
own minimal graph, same shape as apriltag_launch.py.

encoder's `tensor_name` param is set to 'input' and its output topic
remapped to 'tensor_pub' so it lines up with TensorRTNode's default
INPUT_TOPIC_NAME/input_tensor_names -- see dnn_classify_check.py for the
companion image publisher / output check.
"""
import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

THIS_DIR = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    model_file_path = os.path.join(THIS_DIR, 'models', 'mobilenetv2-1.0.onnx')

    encoder_node = ComposableNode(
        name='dnn_image_encoder',
        package='isaac_ros_dnn_image_encoder',
        plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
        namespace='',
        parameters=[{
            'input_image_width': 1920,
            'input_image_height': 1080,
            'network_image_width': 224,
            'network_image_height': 224,
            'input_encoding': 'bgr8',
            'enable_padding': True,
            'tensor_name': 'input',
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
            'engine_file_path': '/tmp/trt_engine.plan',
            'output_binding_names': ['mobilenetv20_output_flatten0_reshape0'],
            'output_tensor_names': ['output'],
            'input_tensor_names': ['input'],
            'input_binding_names': ['data'],
            'verbose': False,
            'force_engine_update': False,
        }],
    )

    container = ComposableNodeContainer(
        package='rclcpp_components',
        name='dnn_classify_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[encoder_node, tensor_rt_node],
        output='both',
        arguments=['--ros-args', '--log-level', 'info'],
    )
    return LaunchDescription([container])
