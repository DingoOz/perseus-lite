"""
Stage 5 decision-gate test: does isaac_ros_tensor_rt (built against the
apt-installed TensorRT 10.16.2 dev libs, CUDA 13.2, on Orin/JetPack7) load
and run a real TensorRT engine build + inference pass end to end?

Loads NVIDIA's own mobilenetv2-1.0.onnx test fixture (from
isaac_ros_tensor_rt/test/models/, git-lfs, copied to ../models/ -- see
tensor_rt_check.py for the companion round-trip publisher/subscriber). Not
NVIDIA's own isaac_ros_tensor_rt_test.py -- that pulls in isaac_ros_test's
IsaacROSBaseTest (heavier deps we've avoided elsewhere in this experiment);
this is our own minimal launch, same shape as nitros_roundtrip_launch.py /
apriltag_launch.py.
"""
import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

THIS_DIR = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    model_file_path = os.path.join(THIS_DIR, 'models', 'mobilenetv2-1.0.onnx')

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
        }],
    )

    container = ComposableNodeContainer(
        package='rclcpp_components',
        name='tensor_rt_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[tensor_rt_node],
        output='both',
        arguments=['--ros-args', '--log-level', 'info'],
    )
    return LaunchDescription([container])
