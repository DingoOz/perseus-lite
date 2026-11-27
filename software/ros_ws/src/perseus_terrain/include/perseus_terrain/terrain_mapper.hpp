#pragma once

/// @file terrain_mapper.hpp
/// @brief The terrain_mapper node: depth camera in, terrain cost grids out.
///
/// Pipeline, per depth frame:
///   depth image --(every Nth pixel)--> points in the optical frame
///     --(robot-frame crops: own body, ceiling)--> points in the global frame (odom).
/// Per update (update_rate_hz), for the frames that arrived since the last one:
///   per-frame mode (point_buffer_s 0, the default): each frame alone
///     --classify_terrain()--> observation --LogOddsMap::fold()--> memory
///   window mode (point_buffer_s > 0): the last point_buffer_s of frames together
///     --classify_terrain()--> observation, folded into memory once per memory period
///   The latest observation is the local grid                      -> /terrain/local_grid
/// Per memory period (memory_rate_hz): memory, with the footprint cleared -> /terrain/global_grid
///
/// Both grids are published in the global frame (odom). odom is continuous, so the
/// memory never jumps when SLAM corrects map -> odom; the TerrainGridLayer costmap
/// plugin transforms the grids into whichever frame a costmap uses.

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>

#include "perseus_terrain/depth_projection.hpp"
#include "perseus_terrain/footprint.hpp"
#include "perseus_terrain/log_odds_map.hpp"
#include "perseus_terrain/terrain_classifier.hpp"

namespace perseus_terrain
{
    /// @brief See the file comment.
    class TerrainMapper : public rclcpp::Node
    {
    public:
        explicit TerrainMapper(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
        /// @brief Sensor-frame points waiting for their transform to become available.
        struct PendingFrame
        {
            rclcpp::Time stamp;
            std::string frame_id;
            std::vector<Point3> points;
        };

        /// @brief Points of one frame, already in the global frame.
        struct BufferedFrame
        {
            rclcpp::Time stamp;
            std::vector<Point3> points;
        };

        /// @brief Robot pose in the global frame, flattened to (x, y, yaw).
        struct Pose2D
        {
            double x;
            double y;
            double yaw;
        };

        void declare_parameters();
        void on_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& message);
        void on_depth(const sensor_msgs::msg::Image::ConstSharedPtr& message);
        void on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message);
        bool accept_frame(const rclcpp::Time& stamp);
        void queue_frame(const std_msgs::msg::Header& header, std::vector<Point3> points);
        void transform_pending_frames();
        void on_update();
        void publish_debug(const TerrainObservation& observation, const std::vector<Point3>& points,
                           const rclcpp::Time& stamp);
        std::optional<Pose2D> robot_pose();
        void clear_under_robot(const Pose2D& pose);
        nav_msgs::msg::OccupancyGrid to_message(const GridGeometry& geometry, const std::vector<std::int8_t>& data,
                                                const rclcpp::Time& stamp) const;

        // Parameters
        std::string global_frame_;
        std::string robot_frame_;
        double resolution_{0.05};
        double point_buffer_s_{0.0};
        double local_half_extent_m_{4.0};
        double max_input_rate_hz_{10.0};
        double max_height_above_robot_m_{1.0};
        std::vector<double> body_crop_min_;
        std::vector<double> body_crop_max_;
        double transform_wait_s_{0.5};
        bool clear_under_robot_{true};
        double clear_radius_m_{1.5};
        bool publish_debug_{false};
        DepthProjectionParams projection_;
        int cloud_point_stride_{1};
        ClassifierParams classifier_;
        Polygon footprint_;

        // State
        std::optional<PinholeIntrinsics> intrinsics_;
        std::optional<rclcpp::Time> last_accepted_stamp_;
        std::deque<PendingFrame> pending_;
        std::deque<BufferedFrame> buffer_;
        bool new_data_since_update_{false};
        bool new_data_since_memory_{false};
        std::unique_ptr<LogOddsMap> memory_;
        rclcpp::Time last_memory_update_;
        double memory_period_s_{1.0};

        // ROS interfaces
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_pub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_pub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_step_up_pub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_step_down_pub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_height_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_points_pub_;
        rclcpp::Service<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_around_robot_srv_;
        rclcpp::Service<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_all_srv_;
        rclcpp::TimerBase::SharedPtr update_timer_;
    };

}
