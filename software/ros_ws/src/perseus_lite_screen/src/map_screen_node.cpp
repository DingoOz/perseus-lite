#include "map_screen_node.hpp"

#include <tf2/utils.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace perseus_lite_screen
{

    MapScreenNode::MapScreenNode()
        : rclcpp::Node("perseus_lite_screen_node")
    {
        _map_frame = declare_parameter<std::string>("map_frame", "map");
        _base_frame = declare_parameter<std::string>("base_frame", "base_link");

        const auto map_topic =
            declare_parameter<std::string>("map_topic", "/map");
        const auto scan_topic =
            declare_parameter<std::string>("scan_topic", "/scan_filtered");
        const auto plan_topic =
            declare_parameter<std::string>("plan_topic", "/plan");
        const auto goal_topic =
            declare_parameter<std::string>("goal_topic", "/goal_pose");

        _tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
        _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

        rclcpp::QoS map_qos(1);
        map_qos.transient_local().reliable();
        _map_sub = create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic, map_qos,
            [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
            {
                _on_map(std::move(msg));
            });

        _scan_sub = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
            {
                _on_scan(std::move(msg));
            });

        _plan_sub = create_subscription<nav_msgs::msg::Path>(
            plan_topic, rclcpp::QoS(1),
            [this](nav_msgs::msg::Path::ConstSharedPtr msg)
            {
                _on_path(std::move(msg));
            });

        _goal_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
            goal_topic, rclcpp::QoS(1),
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
            {
                _on_goal(std::move(msg));
            });

        RCLCPP_INFO(get_logger(),
                    "perseus_lite_screen subscribed: %s, %s, %s, %s (frames %s -> %s)",
                    map_topic.c_str(), scan_topic.c_str(), plan_topic.c_str(),
                    goal_topic.c_str(), _map_frame.c_str(), _base_frame.c_str());
    }

    void MapScreenNode::_on_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        _map = std::move(msg);
        _last_map_stamp = now();
    }

    void MapScreenNode::_on_scan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        _scan = std::move(msg);
    }

    void MapScreenNode::_on_path(nav_msgs::msg::Path::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        _path = std::move(msg);
    }

    void MapScreenNode::_on_goal(
        geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        _goal = std::move(msg);
    }

    nav_msgs::msg::OccupancyGrid::ConstSharedPtr MapScreenNode::latest_map() const
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _map;
    }

    sensor_msgs::msg::LaserScan::ConstSharedPtr MapScreenNode::latest_scan() const
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _scan;
    }

    nav_msgs::msg::Path::ConstSharedPtr MapScreenNode::latest_path() const
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _path;
    }

    geometry_msgs::msg::PoseStamped::ConstSharedPtr MapScreenNode::latest_goal() const
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _goal;
    }

    rclcpp::Time MapScreenNode::last_map_stamp() const
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _last_map_stamp;
    }

    std::optional<RobotPose> MapScreenNode::lookup_robot_pose() const
    {
        geometry_msgs::msg::TransformStamped tf;
        try
        {
            tf = _tf_buffer->lookupTransform(_map_frame, _base_frame,
                                             tf2::TimePointZero);
        }
        catch (const tf2::TransformException&)
        {
            return std::nullopt;
        }
        RobotPose pose;
        pose.x = tf.transform.translation.x;
        pose.y = tf.transform.translation.y;
        pose.yaw = tf2::getYaw(tf.transform.rotation);
        return pose;
    }

    bool MapScreenNode::try_transform_point(const std::string& source_frame,
                                            const rclcpp::Time& stamp,
                                            double in_x, double in_y,
                                            double& out_x, double& out_y) const
    {
        geometry_msgs::msg::PointStamped p_in;
        p_in.header.frame_id = source_frame;
        p_in.header.stamp = stamp;
        p_in.point.x = in_x;
        p_in.point.y = in_y;
        p_in.point.z = 0.0;

        geometry_msgs::msg::PointStamped p_out;
        try
        {
            _tf_buffer->transform(p_in, p_out, _map_frame,
                                  tf2::durationFromSec(0.05));
        }
        catch (const tf2::TransformException&)
        {
            // Fall back to latest available transform.
            try
            {
                p_in.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
                _tf_buffer->transform(p_in, p_out, _map_frame,
                                      tf2::durationFromSec(0.05));
            }
            catch (const tf2::TransformException&)
            {
                return false;
            }
        }
        out_x = p_out.point.x;
        out_y = p_out.point.y;
        return true;
    }

}  // namespace perseus_lite_screen
