#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>

namespace perseus_lite_screen
{

    struct RobotPose
    {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
    };

    class MapScreenNode : public rclcpp::Node
    {
    public:
        MapScreenNode();

        nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_map() const;
        sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan() const;
        nav_msgs::msg::Path::ConstSharedPtr latest_path() const;
        geometry_msgs::msg::PoseStamped::ConstSharedPtr latest_goal() const;

        std::optional<RobotPose> lookup_robot_pose() const;

        bool try_transform_point(const std::string& source_frame,
                                 const rclcpp::Time& stamp, double in_x,
                                 double in_y, double& out_x, double& out_y) const;

        rclcpp::Time last_map_stamp() const;

        std::string map_frame() const { return _map_frame; }
        std::string base_frame() const { return _base_frame; }

    private:
        void _on_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
        void _on_scan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
        void _on_path(nav_msgs::msg::Path::ConstSharedPtr msg);
        void _on_goal(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

        std::string _map_frame;
        std::string _base_frame;

        std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _map_sub;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _scan_sub;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _plan_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _goal_sub;

        mutable std::mutex _data_mutex;
        nav_msgs::msg::OccupancyGrid::ConstSharedPtr _map;
        sensor_msgs::msg::LaserScan::ConstSharedPtr _scan;
        nav_msgs::msg::Path::ConstSharedPtr _path;
        geometry_msgs::msg::PoseStamped::ConstSharedPtr _goal;
        rclcpp::Time _last_map_stamp{0, 0, RCL_ROS_TIME};
    };

}  // namespace perseus_lite_screen
