#include "perseus_terrain/terrain_mapper.hpp"

#include <tf2/exceptions.h>
#include <tf2/utils.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>

namespace perseus_terrain
{
    namespace
    {
        /// @brief Scales a float debug layer onto 0..100: 50 at @p threshold, 100 at twice
        ///        it, -1 where unknown.
        std::vector<std::int8_t> scale_layer(const std::vector<float>& layer, double threshold)
        {
            std::vector<std::int8_t> out(layer.size(), -1);
            if (threshold <= 0.0)
            {
                return out;
            }
            for (std::size_t i = 0; i < layer.size(); ++i)
            {
                if (std::isnan(layer[i]))
                {
                    continue;
                }
                const double scaled = 50.0 * static_cast<double>(layer[i]) / threshold;
                out[i] = static_cast<std::int8_t>(std::clamp(std::lround(scaled), 0L, 100L));
            }
            return out;
        }
    }

    TerrainMapper::TerrainMapper(const rclcpp::NodeOptions& options)
        : rclcpp::Node("terrain_mapper", options)
    {
        declare_parameters();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        const std::string input_type = declare_parameter<std::string>("input_type", "depth");
        if (input_type == "depth")
        {
            camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
                declare_parameter<std::string>("camera_info_topic", "/oak/stereo/camera_info"), rclcpp::SensorDataQoS(),
                std::bind(&TerrainMapper::on_camera_info, this, std::placeholders::_1));
            depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
                declare_parameter<std::string>("depth_topic", "/oak/stereo/image_raw"), rclcpp::SensorDataQoS(),
                std::bind(&TerrainMapper::on_depth, this, std::placeholders::_1));
        }
        else if (input_type == "pointcloud")
        {
            cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                declare_parameter<std::string>("pointcloud_topic", "/oak/points"), rclcpp::SensorDataQoS(),
                std::bind(&TerrainMapper::on_cloud, this, std::placeholders::_1));
        }
        else
        {
            throw std::invalid_argument("input_type must be 'depth' or 'pointcloud', got '" + input_type + "'");
        }

        const rclcpp::QoS local_qos = rclcpp::QoS(1).reliable().durability_volatile();
        // The memory is latched: a costmap or RViz that starts later still gets it at once.
        const rclcpp::QoS global_qos = rclcpp::QoS(1).reliable().transient_local();
        local_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            declare_parameter<std::string>("local_grid_topic", "/terrain/local_grid"), local_qos);
        global_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            declare_parameter<std::string>("global_grid_topic", "/terrain/global_grid"), global_qos);
        if (publish_debug_)
        {
            debug_step_up_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/terrain/debug/step_up", local_qos);
            debug_step_down_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/terrain/debug/step_down", local_qos);
            debug_height_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/terrain/debug/height_above_ground", local_qos);
            debug_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/terrain/debug/points", local_qos);
        }

        clear_around_robot_srv_ = create_service<nav2_msgs::srv::ClearEntireCostmap>(
            "~/clear_around_robot",
            [this](const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Request>,
                   std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Response>)
            {
                const auto pose = robot_pose();
                if (!pose || !memory_)
                {
                    RCLCPP_WARN(get_logger(), "clear_around_robot: no robot pose or no memory yet; nothing cleared");
                    return;
                }
                memory_->forget_disc(pose->x, pose->y, clear_radius_m_);
                RCLCPP_INFO(get_logger(), "Forgot terrain memory within %.2f m of the robot", clear_radius_m_);
                if (!memory_->empty())
                {
                    global_pub_->publish(to_message(memory_->geometry(), memory_->to_cost(), now()));
                }
            });
        clear_all_srv_ = create_service<nav2_msgs::srv::ClearEntireCostmap>(
            "~/clear_all",
            [this](const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Request>,
                   std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Response>)
            {
                if (!memory_)
                {
                    return;
                }
                memory_->reset();
                RCLCPP_INFO(get_logger(), "Forgot all terrain memory");
                if (!memory_->empty())
                {
                    global_pub_->publish(to_message(memory_->geometry(), memory_->to_cost(), now()));
                }
            });

        const double update_rate_hz = std::max(0.1, get_parameter("update_rate_hz").as_double());
        update_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / update_rate_hz),
                                          std::bind(&TerrainMapper::on_update, this));
        last_memory_update_ = now();

        RCLCPP_INFO(get_logger(), "terrain_mapper: %s input, %.3f m cells, grids in '%s'", input_type.c_str(), resolution_,
                    global_frame_.c_str());
    }

    void TerrainMapper::declare_parameters()
    {
        global_frame_ = declare_parameter<std::string>("global_frame", "odom");
        robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
        resolution_ = declare_parameter<double>("resolution_m", 0.05);
        point_buffer_s_ = declare_parameter<double>("point_buffer_s", 0.0);
        local_half_extent_m_ = declare_parameter<double>("local_half_extent_m", 4.0);
        declare_parameter<double>("update_rate_hz", 5.0);
        memory_period_s_ = 1.0 / std::max(0.01, declare_parameter<double>("memory_rate_hz", 1.0));
        max_input_rate_hz_ = declare_parameter<double>("max_input_rate_hz", 10.0);
        transform_wait_s_ = declare_parameter<double>("transform_wait_s", 0.5);
        max_height_above_robot_m_ = declare_parameter<double>("max_height_above_robot_m", 1.0);
        // The robot's own body, in robot_frame. Points inside are dropped before
        // classification. Defaults: the lite footprint, from the ground to above the camera.
        body_crop_min_ = declare_parameter<std::vector<double>>("body_crop_min", {-0.45, -0.30, -0.10});
        body_crop_max_ = declare_parameter<std::vector<double>>("body_crop_max", {0.25, 0.30, 0.50});
        clear_under_robot_ = declare_parameter<bool>("clear_under_robot", true);
        clear_radius_m_ = declare_parameter<double>("clear_radius_m", 1.5);
        publish_debug_ = declare_parameter<bool>("publish_debug", false);

        projection_.pixel_stride = static_cast<int>(declare_parameter<int>("pixel_stride", 4));
        projection_.min_depth_m = declare_parameter<double>("min_depth_m", 0.3);
        projection_.max_depth_m = declare_parameter<double>("max_depth_m", 3.5);
        cloud_point_stride_ = static_cast<int>(declare_parameter<int>("cloud_point_stride", 1));

        classifier_.ground_window_m = declare_parameter<double>("classifier.ground_window_m", classifier_.ground_window_m);
        classifier_.max_slope_deg = declare_parameter<double>("classifier.max_slope_deg", classifier_.max_slope_deg);
        classifier_.max_step_up_m = declare_parameter<double>("classifier.max_step_up_m", classifier_.max_step_up_m);
        classifier_.max_step_down_m = declare_parameter<double>("classifier.max_step_down_m", classifier_.max_step_down_m);
        classifier_.max_height_above_ground_m =
            declare_parameter<double>("classifier.max_height_above_ground_m", classifier_.max_height_above_ground_m);
        classifier_.overhead_clearance_m =
            declare_parameter<double>("classifier.overhead_clearance_m", classifier_.overhead_clearance_m);
        classifier_.min_points_per_cell =
            static_cast<int>(declare_parameter<int>("classifier.min_points_per_cell", classifier_.min_points_per_cell));
        classifier_.min_obstacle_cells =
            static_cast<int>(declare_parameter<int>("classifier.min_obstacle_cells", classifier_.min_obstacle_cells));
        classifier_.floor_percentile = declare_parameter<double>("classifier.floor_percentile", classifier_.floor_percentile);
        classifier_.top_percentile = declare_parameter<double>("classifier.top_percentile", classifier_.top_percentile);
        classifier_.rough_start_fraction =
            declare_parameter<double>("classifier.rough_start_fraction", classifier_.rough_start_fraction);
        classifier_.rough_max_cost =
            static_cast<int>(declare_parameter<int>("classifier.rough_max_cost", classifier_.rough_max_cost));

        MemoryParams memory;
        memory.log_odds_hit = declare_parameter<double>("memory.log_odds_hit", memory.log_odds_hit);
        memory.log_odds_miss = declare_parameter<double>("memory.log_odds_miss", memory.log_odds_miss);
        memory.log_odds_min = declare_parameter<double>("memory.log_odds_min", memory.log_odds_min);
        memory.log_odds_max = declare_parameter<double>("memory.log_odds_max", memory.log_odds_max);
        memory.occupied_threshold = declare_parameter<double>("memory.occupied_threshold", memory.occupied_threshold);
        memory.initial_size_m = declare_parameter<double>("memory.initial_size_m", memory.initial_size_m);
        memory.growth_step_m = declare_parameter<double>("memory.growth_step_m", memory.growth_step_m);
        memory.max_size_m = declare_parameter<double>("memory.max_size_m", memory.max_size_m);
        memory_ = std::make_unique<LogOddsMap>(resolution_, memory);

        const double padding = declare_parameter<double>("footprint_padding_m", 0.05);
        footprint_ = pad_polygon(polygon_from_flat(declare_parameter<std::vector<double>>(
                                     "footprint", {0.22, 0.27, 0.22, -0.27, -0.42, -0.27, -0.42, 0.27})),
                                 padding);

        if (resolution_ <= 0.0 || body_crop_min_.size() != 3 || body_crop_max_.size() != 3)
        {
            throw std::invalid_argument("resolution_m must be > 0 and body_crop_min/max must have 3 values");
        }
        if (footprint_.empty())
        {
            RCLCPP_WARN(get_logger(),
                        "footprint is not a flat [x0, y0, x1, y1, ...] list of >= 3 points; "
                        "the cells under the robot will not be cleared");
        }
    }

    void TerrainMapper::on_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& message)
    {
        // Rectified images are described by P; fall back to K if a driver leaves P empty.
        PinholeIntrinsics intrinsics;
        if (message->p[0] > 0.0 && message->p[5] > 0.0)
        {
            intrinsics = {message->p[0], message->p[5], message->p[2], message->p[6]};
        }
        else
        {
            intrinsics = {message->k[0], message->k[4], message->k[2], message->k[5]};
        }
        if (!intrinsics_)
        {
            RCLCPP_INFO(get_logger(), "Camera intrinsics: fx %.1f fy %.1f cx %.1f cy %.1f (%ux%u)", intrinsics.fx,
                        intrinsics.fy, intrinsics.cx, intrinsics.cy, message->width, message->height);
        }
        intrinsics_ = intrinsics;
    }

    bool TerrainMapper::accept_frame(const rclcpp::Time& stamp)
    {
        if (max_input_rate_hz_ > 0.0 && last_accepted_stamp_ && stamp.get_clock_type() == last_accepted_stamp_->get_clock_type())
        {
            const double since = (stamp - *last_accepted_stamp_).seconds();
            // Allow a little early so a 10 Hz cap does not drop every other frame of a
            // 10 Hz camera because of jitter.
            if (since >= 0.0 && since < 0.9 / max_input_rate_hz_)
            {
                return false;
            }
        }
        last_accepted_stamp_ = stamp;
        return true;
    }

    void TerrainMapper::on_depth(const sensor_msgs::msg::Image::ConstSharedPtr& message)
    {
        if (!intrinsics_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Depth image received but no camera_info yet");
            return;
        }
        if (!accept_frame(rclcpp::Time(message->header.stamp, get_clock()->get_clock_type())))
        {
            return;
        }
        const auto encoding = depth_encoding_from_string(message->encoding);
        if (!encoding)
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                                  "Unsupported depth encoding '%s' (expected 16UC1 or 32FC1)", message->encoding.c_str());
            return;
        }
        DepthImageView view;
        view.data = message->data.data();
        view.step = message->step;
        view.width = static_cast<int>(message->width);
        view.height = static_cast<int>(message->height);
        view.encoding = *encoding;
        view.big_endian = message->is_bigendian != 0;
        if (message->data.size() < static_cast<std::size_t>(message->step) * message->height)
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Depth image buffer is shorter than step x height");
            return;
        }
        queue_frame(message->header, project_depth(view, *intrinsics_, projection_));
    }

    void TerrainMapper::on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message)
    {
        if (!accept_frame(rclcpp::Time(message->header.stamp, get_clock()->get_clock_type())))
        {
            return;
        }
        std::vector<Point3> points;
        try
        {
            sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
            sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
            sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
            const std::size_t total = static_cast<std::size_t>(message->width) * message->height;
            const auto stride = static_cast<std::size_t>(std::max(1, cloud_point_stride_));
            const double min_squared = projection_.min_depth_m * projection_.min_depth_m;
            const double max_squared = projection_.max_depth_m * projection_.max_depth_m;
            points.reserve(total / stride + 1);
            for (std::size_t i = 0; i < total; i += stride, x += static_cast<int>(stride), y += static_cast<int>(stride),
                             z += static_cast<int>(stride))
            {
                if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z))
                {
                    continue;
                }
                const double range_squared = static_cast<double>(*x) * *x + static_cast<double>(*y) * *y +
                                             static_cast<double>(*z) * *z;
                if (range_squared < min_squared || range_squared > max_squared)
                {
                    continue;
                }
                points.push_back({*x, *y, *z});
            }
        }
        catch (const std::runtime_error& error)
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "PointCloud2 has no float x/y/z fields: %s",
                                  error.what());
            return;
        }
        queue_frame(message->header, std::move(points));
    }

    void TerrainMapper::queue_frame(const std_msgs::msg::Header& header, std::vector<Point3> points)
    {
        if (points.empty())
        {
            return;
        }
        pending_.push_back({rclcpp::Time(header.stamp, get_clock()->get_clock_type()), header.frame_id, std::move(points)});
        // A burst of frames while TF is missing must not grow without bound.
        while (pending_.size() > 50)
        {
            pending_.pop_front();
        }
    }

    void TerrainMapper::transform_pending_frames()
    {
        const rclcpp::Time current = now();
        while (!pending_.empty())
        {
            PendingFrame& frame = pending_.front();
            geometry_msgs::msg::TransformStamped global_from_sensor;
            geometry_msgs::msg::TransformStamped robot_from_sensor;
            try
            {
                if (!tf_buffer_->canTransform(global_frame_, frame.frame_id, frame.stamp))
                {
                    // TF for this stamp is usually just behind the image; wait for it a
                    // little, then give up on the frame rather than stall the queue.
                    if ((current - frame.stamp).seconds() < transform_wait_s_)
                    {
                        return;
                    }
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                         "No transform %s <- %s at the frame's stamp; dropping depth frames",
                                         global_frame_.c_str(), frame.frame_id.c_str());
                    pending_.pop_front();
                    continue;
                }
                global_from_sensor = tf_buffer_->lookupTransform(global_frame_, frame.frame_id, frame.stamp);
                robot_from_sensor = tf_buffer_->lookupTransform(robot_frame_, frame.frame_id, tf2::TimePointZero);
            }
            catch (const tf2::TransformException& error)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping a depth frame: %s", error.what());
                pending_.pop_front();
                continue;
            }

            const Eigen::Isometry3f to_global = tf2::transformToEigen(global_from_sensor).cast<float>();
            const Eigen::Isometry3f to_robot = tf2::transformToEigen(robot_from_sensor).cast<float>();
            const auto crop_min = Eigen::Vector3f(static_cast<float>(body_crop_min_[0]), static_cast<float>(body_crop_min_[1]),
                                                  static_cast<float>(body_crop_min_[2]));
            const auto crop_max = Eigen::Vector3f(static_cast<float>(body_crop_max_[0]), static_cast<float>(body_crop_max_[1]),
                                                  static_cast<float>(body_crop_max_[2]));
            const auto ceiling = static_cast<float>(max_height_above_robot_m_);

            BufferedFrame buffered;
            buffered.stamp = frame.stamp;
            buffered.points.reserve(frame.points.size());
            for (const Point3& point : frame.points)
            {
                const Eigen::Vector3f sensor(point.x, point.y, point.z);
                // Crops are applied in the robot frame: they describe the robot, and a
                // height cut taken there does not move with odometry's z drift.
                const Eigen::Vector3f in_robot = to_robot * sensor;
                if (in_robot.z() > ceiling)
                {
                    continue;
                }
                if ((in_robot.array() >= crop_min.array()).all() && (in_robot.array() <= crop_max.array()).all())
                {
                    continue;
                }
                const Eigen::Vector3f in_global = to_global * sensor;
                buffered.points.push_back({in_global.x(), in_global.y(), in_global.z()});
            }
            pending_.pop_front();
            if (!buffered.points.empty())
            {
                buffer_.push_back(std::move(buffered));
                new_data_since_update_ = true;
                new_data_since_memory_ = true;
            }
        }
    }

    std::optional<TerrainMapper::Pose2D> TerrainMapper::robot_pose()
    {
        try
        {
            const auto transform = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
            return Pose2D{transform.transform.translation.x, transform.transform.translation.y,
                          tf2::getYaw(transform.transform.rotation)};
        }
        catch (const tf2::TransformException& error)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No robot pose (%s <- %s): %s", global_frame_.c_str(),
                                 robot_frame_.c_str(), error.what());
            return std::nullopt;
        }
    }

    void TerrainMapper::clear_under_robot(const Pose2D& pose)
    {
        if (footprint_.empty())
        {
            return;
        }
        memory_->mark_free(transform_polygon(footprint_, pose.x, pose.y, pose.yaw));
    }

    void TerrainMapper::on_update()
    {
        transform_pending_frames();
        if (buffer_.empty() || !new_data_since_update_)
        {
            // Nothing new: re-classifying the same points would add nothing, and folding
            // them into memory again would count the same evidence twice.
            return;
        }
        new_data_since_update_ = false;
        const auto pose = robot_pose();
        if (!pose)
        {
            return;
        }
        const rclcpp::Time newest = buffer_.back().stamp;
        const GridGeometry window = centred_window(pose->x, pose->y, local_half_extent_m_, resolution_);

        if (point_buffer_s_ <= 0.0)
        {
            // PER-FRAME MODE (the lite default). Heights are only ever compared within one
            // depth frame, where every point shares one camera pose. The lite's EKF is 2D:
            // it cannot see the chassis bounce, pitch or roll on its rockers, so two frames
            // a few hundred ms apart disagree about the height of the same flat ground by
            // centimetres, which reads as steps if they are classified together. Temporal
            // fusion happens only in memory, which counts verdicts and never compares
            // heights across frames. Every new frame is classified and folded once.
            TerrainObservation observation;
            for (const BufferedFrame& frame : buffer_)
            {
                observation = classify_terrain(frame.points, window, classifier_);
                memory_->fold(observation);
            }
            local_pub_->publish(to_message(window, observation.cost, newest));
            if (publish_debug_)
            {
                publish_debug(observation, buffer_.back().points, newest);
            }
            buffer_.clear();
        }
        else
        {
            // WINDOW MODE: classify the last point_buffer_s of frames together. Denser,
            // but only valid when TF tracks the camera's full 3D motion (a 6-DOF state
            // estimate); see point_buffer_s in config/terrain_mapper.yaml.
            while (!buffer_.empty() && (newest - buffer_.front().stamp).seconds() > point_buffer_s_)
            {
                buffer_.pop_front();
            }
            std::size_t total = 0;
            for (const BufferedFrame& frame : buffer_)
            {
                total += frame.points.size();
            }
            std::vector<Point3> points;
            points.reserve(total);
            for (const BufferedFrame& frame : buffer_)
            {
                points.insert(points.end(), frame.points.begin(), frame.points.end());
            }
            const TerrainObservation observation = classify_terrain(points, window, classifier_);
            local_pub_->publish(to_message(window, observation.cost, newest));
            if (publish_debug_)
            {
                publish_debug(observation, points, newest);
            }
            // The window overlaps itself between updates, so fold only once per memory
            // period (with memory_rate_hz ~ 1 / point_buffer_s each frame counts ~once).
            if (new_data_since_memory_ && (now() - last_memory_update_).seconds() >= memory_period_s_)
            {
                memory_->fold(observation);
                new_data_since_memory_ = false;
            }
        }

        // Publish memory at memory_rate_hz.
        const rclcpp::Time current = now();
        if ((current - last_memory_update_).seconds() >= memory_period_s_)
        {
            last_memory_update_ = current;
            if (clear_under_robot_)
            {
                clear_under_robot(*pose);
            }
            if (!memory_->empty())
            {
                global_pub_->publish(to_message(memory_->geometry(), memory_->to_cost(), newest));
            }
        }
    }

    void TerrainMapper::publish_debug(const TerrainObservation& observation, const std::vector<Point3>& points,
                                      const rclcpp::Time& stamp)
    {
        const GridGeometry& geometry = observation.geometry;
        debug_step_up_pub_->publish(to_message(geometry, scale_layer(observation.step_up, classifier_.max_step_up_m), stamp));
        debug_step_down_pub_->publish(
            to_message(geometry, scale_layer(observation.step_down, classifier_.max_step_down_m), stamp));
        debug_height_pub_->publish(to_message(
            geometry, scale_layer(observation.height_above_ground, classifier_.max_height_above_ground_m), stamp));

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.frame_id = global_frame_;
        cloud.header.stamp = stamp;
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(points.size());
        sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
        for (const Point3& point : points)
        {
            *x = point.x;
            *y = point.y;
            *z = point.z;
            ++x;
            ++y;
            ++z;
        }
        debug_points_pub_->publish(cloud);
    }

    nav_msgs::msg::OccupancyGrid TerrainMapper::to_message(const GridGeometry& geometry, const std::vector<std::int8_t>& data,
                                                           const rclcpp::Time& stamp) const
    {
        nav_msgs::msg::OccupancyGrid grid;
        grid.header.frame_id = global_frame_;
        grid.header.stamp = stamp;
        grid.info.map_load_time = stamp;
        grid.info.resolution = static_cast<float>(geometry.resolution);
        grid.info.width = static_cast<std::uint32_t>(geometry.width);
        grid.info.height = static_cast<std::uint32_t>(geometry.height);
        grid.info.origin.position.x = geometry.origin_x;
        grid.info.origin.position.y = geometry.origin_y;
        grid.info.origin.orientation.w = 1.0;
        grid.data = data;
        return grid;
    }

}
