"""
Stage 12: occupancy-grid global localization -- the first capability in
this whole from-source experiment matched to the robot's actual 2D lidar
(via NitrosFlatScan, a 2D-LaserScan-equivalent NITROS type) rather than
its camera. Genuinely new CUDA code too:
occupancy_grid_localizer_gpu.cu does GPU-parallel batch scan-matching
(scoring many candidate poses against the map in parallel), unlike
Stage 5's pure-TensorRT-API node or Stage 11's TensorRT-based ESS.

Not redundant with the robot's existing slam_toolbox: slam_toolbox does
SLAM (builds the map); this does relocalization (finds the robot's pose
*within* an already-built map) -- an AMCL-style capability, but
GPU-accelerated grid-search rather than particle-filter-based.

Loads NVIDIA's own real map fixture
(isaac_ros_occupancy_grid_localizer/maps/map.yaml) at startup -- a real
occupancy grid, not a placeholder. A companion rosbag
(data/rosbags/flatscan, 12 real FlatScan messages) is played back by
ogl_check.py's launch wrapper; localization is triggered via the
trigger_grid_search_localization service once a scan is buffered. See
ogl_check.py for the expected-pose assertions (NVIDIA's own ground
truth, not chosen by us).
"""
import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

THIS_DIR = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    map_yaml_path = os.path.join(
        THIS_DIR, 'isaac_ros_mapping_and_localization',
        'isaac_ros_occupancy_grid_localizer', 'maps', 'map.yaml')

    localizer_node = ComposableNode(
        package='isaac_ros_occupancy_grid_localizer',
        plugin='nvidia::isaac_ros::occupancy_grid_localizer::OccupancyGridLocalizerNode',
        name='occupancy_grid_localizer',
        namespace='',
        parameters=[
            # map_yaml_path passed *both* ways, matching NVIDIA's own test:
            # (1) as the raw map.yaml file itself -- ROS 2 launch treats a
            # bare .yaml list entry as a *parameters file*, so this is what
            # actually populates the `image`/`resolution`/`origin` params
            # from the map.yaml's own keys (nav2 map.yaml format happens to
            # use the same key names this node declares as parameters);
            # (2) as the `map_yaml_path` string param the node also reads
            # directly to compute map_png_path's directory component.
            map_yaml_path,
            {
                'loc_result_frame': 'map',
                'map_yaml_path': map_yaml_path,
            },
        ],
    )

    container = ComposableNodeContainer(
        package='rclcpp_components',
        name='ogl_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[localizer_node],
        output='both',
        arguments=['--ros-args', '--log-level', 'info'],
    )
    return LaunchDescription([container])
