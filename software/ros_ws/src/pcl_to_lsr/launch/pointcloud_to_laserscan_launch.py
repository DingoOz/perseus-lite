import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('pcl_to_lsr'),
        'config',
        'pointcloud_to_laserscan.yaml',
    )

    # Topic arguments. The node's own topic names are relative ("cloud_in",
    # "scan_out"), so these remappings take effect. The output default is not
    # /scan, so it can't collide with the robot's real 2D lidar.
    declare_publisher_name = DeclareLaunchArgument(
        'scan_out', default_value='cloud_scan', description='Output LaserScan topic'
    )
    declare_subscriber_name = DeclareLaunchArgument(
        'scan_in', default_value='cloud_in', description='Input PointCloud2 topic'
    )

    return LaunchDescription(
        [
            declare_subscriber_name,
            declare_publisher_name,
            Node(
                package='pcl_to_lsr',
                executable='pointcloud_to_laserscan_node',
                remappings=[
                    ('cloud_in', LaunchConfiguration('scan_in')),
                    ('scan_out', LaunchConfiguration('scan_out')),
                ],
                parameters=[config_file],
                name='pointcloud_to_laserscan',
            ),
        ]
    )
