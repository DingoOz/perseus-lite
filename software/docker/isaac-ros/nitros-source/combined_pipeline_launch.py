"""
Stage 10: every capability built in Stages 4-8 has only ever been proven
running ALONE in its own container. Nothing has tested whether they can
coexist -- four TensorRT engines (or three TensorRT chains plus VPI/
cuAprilTag) sharing one GPU, one CUDA context, one component_container_mt
process, without resource contention or topic collisions. That's a real
prerequisite before any of this could plausibly run together on the robot.

This launches AprilTag + YOLOv8 + U-Net + CenterPose in ONE container,
each in its own ROS namespace so their identically-named relative topics
("image", "tensors", "tensor_pub", "tensor_sub", ...) don't collide --
confirmed each pipeline's hardcoded topic names in prior stages (Stage 5's
topic-name bug is exactly why this matters here). Each ComposableNode's
`name` is also made unique per pipeline; component_container_mt requires
unique names within one container.

All parameters are copied verbatim from the individual Stage 4/5-follow-up/
7/8 launch files (apriltag_launch.py, yolov8_launch.py, unet_launch.py,
centerpose_launch.py) -- nothing new is chosen here, this only tests
composition. See combined_pipeline_check.py.
"""
import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

THIS_DIR = os.path.dirname(os.path.abspath(__file__))


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
    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        namespace='/apriltag',
        parameters=[{'size': 0.22, 'max_tags': 64, 'tile_size': 4}],
    )

    yolov8_encoder = ComposableNode(
        name='yolov8_encoder',
        package='isaac_ros_dnn_image_encoder',
        plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
        namespace='/yolov8',
        parameters=[{
            'input_image_width': 640,
            'input_image_height': 640,
            'network_image_width': 640,
            'network_image_height': 640,
            'input_encoding': 'rgb8',
            'image_mean': [0.5, 0.6, 0.25],
            'image_stddev': [0.25, 0.8, 0.5],
            'enable_padding': True,
            'tensor_name': 'input_tensor',
        }],
        remappings=[('tensors', 'tensor_pub')],
    )
    yolov8_tensor_rt = ComposableNode(
        name='yolov8_tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        namespace='/yolov8',
        parameters=[{
            'model_file_path': os.path.join(THIS_DIR, 'models', 'dummy_yolov8s.onnx'),
            'engine_file_path': '/tmp/dummy_yolov8s.plan',
            'output_binding_names': ['output0'],
            'output_tensor_names': ['output_tensor'],
            'input_tensor_names': ['input_tensor'],
            'input_binding_names': ['images'],
            'verbose': False,
            'force_engine_update': False,
        }],
    )
    yolov8_decoder = ComposableNode(
        name='yolov8_decoder',
        package='isaac_ros_yolov8',
        plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
        namespace='/yolov8',
        parameters=[{
            'tensor_name': 'output_tensor',
            'confidence_threshold': 0.25,
            'nms_threshold': 0.45,
        }],
    )

    unet_encoder = ComposableNode(
        name='unet_encoder',
        package='isaac_ros_dnn_image_encoder',
        plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
        namespace='/unet',
        parameters=[{
            'input_image_width': 1200,
            'input_image_height': 632,
            'network_image_width': 960,
            'network_image_height': 544,
            'input_encoding': 'rgb8',
            'enable_padding': True,
            'tensor_name': 'input_tensor',
        }],
        remappings=[('tensors', 'tensor_pub')],
    )
    unet_tensor_rt = ComposableNode(
        name='unet_tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        namespace='/unet',
        parameters=[{
            'model_file_path': os.path.join(THIS_DIR, 'models', 'model.dummy.onnx'),
            'engine_file_path': '/tmp/unet_dummy.plan',
            'input_tensor_names': ['input_tensor'],
            'input_binding_names': ['input_1'],
            'output_tensor_names': ['output_tensor'],
            'output_binding_names': ['softmax_1'],
            'verbose': False,
            'force_engine_update': False,
        }],
    )
    unet_decoder = ComposableNode(
        name='unet_decoder',
        package='isaac_ros_unet',
        plugin='nvidia::isaac_ros::unet::UNetDecoderNode',
        namespace='/unet',
        parameters=[{
            'color_segmentation_mask_encoding': 'rgb8',
            'color_palette': generate_random_color_palette(20),
            'mask_width': 960,
            'mask_height': 544,
        }],
    )

    centerpose_encoder = ComposableNode(
        name='centerpose_encoder',
        package='isaac_ros_dnn_image_encoder',
        plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
        namespace='/centerpose',
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
    centerpose_tensor_rt = ComposableNode(
        name='centerpose_tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        namespace='/centerpose',
        parameters=[{
            'model_file_path': os.path.join(THIS_DIR, 'models', 'centerpose_shoe.onnx'),
            'engine_file_path': '/tmp/centerpose_shoe.plan',
            'input_tensor_names': ['input_tensor'],
            'input_binding_names': ['input'],
            'input_tensor_formats': ['nitros_tensor_list_nchw_rgb_f32'],
            'output_tensor_names': [
                'bboxes', 'scores', 'kps', 'clses',
                'obj_scale', 'kps_displacement_mean', 'kps_heatmap_mean',
            ],
            'output_binding_names': [
                'bboxes', 'scores', 'kps', 'clses',
                'obj_scale', 'kps_displacement_mean', 'kps_heatmap_mean',
            ],
            'output_tensor_formats': ['nitros_tensor_list_nhwc_rgb_f32'],
            'verbose': False,
            'force_engine_update': False,
            'max_workspace_size': 512 * 1024 * 1024,
        }],
    )
    centerpose_decoder = ComposableNode(
        name='centerpose_decoder',
        package='isaac_ros_centerpose',
        plugin='nvidia::isaac_ros::centerpose::CenterPoseDecoderNode',
        namespace='/centerpose',
        parameters=[{
            'output_field_size': [128, 128],
            'cuboid_scaling_factor': 1.0,
            'score_threshold': 0.3,
            'object_name': 'shoe',
        }],
    )

    container = ComposableNodeContainer(
        package='rclcpp_components',
        name='combined_pipeline_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            apriltag_node,
            yolov8_encoder, yolov8_tensor_rt, yolov8_decoder,
            unet_encoder, unet_tensor_rt, unet_decoder,
            centerpose_encoder, centerpose_tensor_rt, centerpose_decoder,
        ],
        output='both',
        arguments=['--ros-args', '--log-level', 'info'],
    )
    return LaunchDescription([container])
