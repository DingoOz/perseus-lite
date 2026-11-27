#include "perseus_terrain/terrain_grid_layer.hpp"

#include <tf2/exceptions.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "perseus_terrain/cost_mapping.hpp"

namespace perseus_terrain
{
    void TerrainGridLayer::onInitialize()
    {
        auto node = node_.lock();
        if (!node)
        {
            throw std::runtime_error("TerrainGridLayer: parent node expired");
        }
        declareParameter("enabled", rclcpp::ParameterValue(true));
        declareParameter("topic", rclcpp::ParameterValue(std::string("/terrain/global_grid")));
        declareParameter("transient_local", rclcpp::ParameterValue(true));
        declareParameter("max_age_s", rclcpp::ParameterValue(0.0));
        declareParameter("lethal_threshold", rclcpp::ParameterValue(100));
        node->get_parameter(getFullName("enabled"), enabled_);
        node->get_parameter(getFullName("topic"), topic_);
        node->get_parameter(getFullName("transient_local"), transient_local_);
        node->get_parameter(getFullName("max_age_s"), max_age_s_);
        node->get_parameter(getFullName("lethal_threshold"), lethal_threshold_);

        // Durability must match the publisher or nothing is delivered and nothing is
        // logged: a transient-local subscriber never matches a volatile publisher.
        rclcpp::QoS qos = rclcpp::QoS(1).reliable();
        if (transient_local_)
        {
            qos.transient_local();
        }
        else
        {
            qos.durability_volatile();
        }
        subscription_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            topic_, qos, std::bind(&TerrainGridLayer::on_grid, this, std::placeholders::_1));
        current_ = true;
        RCLCPP_INFO(logger_, "%s: terrain grid from '%s' (%s, max age %.2f s)", name_.c_str(), topic_.c_str(),
                    transient_local_ ? "transient local" : "volatile", max_age_s_);
    }

    void TerrainGridLayer::on_grid(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& message)
    {
        const std::size_t expected = static_cast<std::size_t>(message->info.width) * message->info.height;
        if (message->info.resolution <= 0.0F || message->data.size() != expected)
        {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "%s: ignoring malformed grid on '%s'", name_.c_str(),
                                 topic_.c_str());
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        grid_frame_ = message->header.frame_id;
        received_ = clock_->now();
        grid_origin_x_ = message->info.origin.position.x;
        grid_origin_y_ = message->info.origin.position.y;
        grid_origin_yaw_ = tf2::getYaw(message->info.origin.orientation);
        geometry_.origin_x = 0.0;
        geometry_.origin_y = 0.0;
        geometry_.resolution = message->info.resolution;
        geometry_.width = static_cast<int>(message->info.width);
        geometry_.height = static_cast<int>(message->info.height);
        data_ = message->data;
        have_grid_ = true;
    }

    bool TerrainGridLayer::grid_is_usable()
    {
        if (!have_grid_)
        {
            return false;
        }
        if (max_age_s_ > 0.0 && (clock_->now() - received_).seconds() > max_age_s_)
        {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 10000, "%s: grid on '%s' is older than %.2f s; ignoring it",
                                 name_.c_str(), topic_.c_str(), max_age_s_);
            return false;
        }
        return true;
    }

    void TerrainGridLayer::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double* min_x,
                                        double* min_y, double* max_x, double* max_y)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        costmap_to_grid_.reset();
        if (!enabled_ || !grid_is_usable())
        {
            // Report the area last written once more, so the master grid resets it and
            // costs from an expired or disabled grid do not linger there.
            if (have_applied_bounds_)
            {
                *min_x = std::min(*min_x, applied_min_x_);
                *min_y = std::min(*min_y, applied_min_y_);
                *max_x = std::max(*max_x, applied_max_x_);
                *max_y = std::max(*max_y, applied_max_y_);
                have_applied_bounds_ = false;
            }
            return;
        }

        const std::string& costmap_frame = layered_costmap_->getGlobalFrameID();
        geometry_msgs::msg::TransformStamped grid_from_costmap;
        try
        {
            grid_from_costmap = tf_->lookupTransform(grid_frame_, costmap_frame, tf2::TimePointZero);
        }
        catch (const tf2::TransformException& error)
        {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "%s: no transform %s <- %s: %s", name_.c_str(),
                                 grid_frame_.c_str(), costmap_frame.c_str(), error.what());
            return;
        }

        // costmap point -> grid frame (TF), then grid frame -> cell frame (subtract the
        // grid origin, undo its yaw). Folded into one planar rotation + translation.
        const double tf_yaw = tf2::getYaw(grid_from_costmap.transform.rotation);
        const double tf_x = grid_from_costmap.transform.translation.x;
        const double tf_y = grid_from_costmap.transform.translation.y;
        const double yaw = tf_yaw - grid_origin_yaw_;
        const double c_origin = std::cos(-grid_origin_yaw_);
        const double s_origin = std::sin(-grid_origin_yaw_);
        const double shifted_x = tf_x - grid_origin_x_;
        const double shifted_y = tf_y - grid_origin_y_;
        costmap_to_grid_ = PlanarTransform{std::cos(yaw), std::sin(yaw), c_origin * shifted_x - s_origin * shifted_y,
                                           s_origin * shifted_x + c_origin * shifted_y};

        // Bounds: the grid's four corners, taken back into the costmap frame.
        const PlanarTransform& forward = *costmap_to_grid_;
        const double extent_x = geometry_.max_x();
        const double extent_y = geometry_.max_y();
        double lo_x = std::numeric_limits<double>::max();
        double lo_y = std::numeric_limits<double>::max();
        double hi_x = std::numeric_limits<double>::lowest();
        double hi_y = std::numeric_limits<double>::lowest();
        for (const auto& [gx, gy] : {std::pair{0.0, 0.0}, std::pair{extent_x, 0.0}, std::pair{0.0, extent_y},
                                     std::pair{extent_x, extent_y}})
        {
            // Inverse of p_grid = R p_costmap + t is p_costmap = R^T (p_grid - t).
            const double dx = gx - forward.tx;
            const double dy = gy - forward.ty;
            const double cx = forward.cos_yaw * dx + forward.sin_yaw * dy;
            const double cy = -forward.sin_yaw * dx + forward.cos_yaw * dy;
            lo_x = std::min(lo_x, cx);
            lo_y = std::min(lo_y, cy);
            hi_x = std::max(hi_x, cx);
            hi_y = std::max(hi_y, cy);
        }
        // Include what was written last time, so cells the grid no longer covers reset.
        if (have_applied_bounds_)
        {
            *min_x = std::min(*min_x, applied_min_x_);
            *min_y = std::min(*min_y, applied_min_y_);
            *max_x = std::max(*max_x, applied_max_x_);
            *max_y = std::max(*max_y, applied_max_y_);
        }
        *min_x = std::min(*min_x, lo_x);
        *min_y = std::min(*min_y, lo_y);
        *max_x = std::max(*max_x, hi_x);
        *max_y = std::max(*max_y, hi_y);
        applied_min_x_ = lo_x;
        applied_min_y_ = lo_y;
        applied_max_x_ = hi_x;
        applied_max_y_ = hi_y;
        have_applied_bounds_ = true;
    }

    void TerrainGridLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                       int max_j)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!enabled_ || !costmap_to_grid_)
        {
            return;
        }
        const PlanarTransform& transform = *costmap_to_grid_;
        const double cell_size = master_grid.getResolution();
        const double extent_x = geometry_.max_x();
        const double extent_y = geometry_.max_y();
        for (int j = min_j; j < max_j; ++j)
        {
            for (int i = min_i; i < max_i; ++i)
            {
                double wx = 0.0;
                double wy = 0.0;
                master_grid.mapToWorld(static_cast<unsigned int>(i), static_cast<unsigned int>(j), wx, wy);
                const double gx = transform.cos_yaw * wx - transform.sin_yaw * wy + transform.tx;
                const double gy = transform.sin_yaw * wx + transform.cos_yaw * wy + transform.ty;
                if (gx < -cell_size || gy < -cell_size || gx > extent_x + cell_size || gy > extent_y + cell_size)
                {
                    continue;
                }
                const std::int8_t value = sample_worst(geometry_, data_, gx, gy, cell_size);
                const std::uint8_t incoming = occupancy_to_cost(value, lethal_threshold_);
                if (incoming == costmap_value::NO_INFORMATION)
                {
                    continue;
                }
                const auto ui = static_cast<unsigned int>(i);
                const auto uj = static_cast<unsigned int>(j);
                master_grid.setCost(ui, uj, combine_max(master_grid.getCost(ui, uj), incoming));
            }
        }
    }

    void TerrainGridLayer::reset()
    {
        // The grid is owned by terrain_mapper; dropping our copy would only blank the
        // layer until the next message. See the file comment.
        current_ = true;
    }

    bool TerrainGridLayer::isClearable()
    {
        return false;
    }

}

PLUGINLIB_EXPORT_CLASS(perseus_terrain::TerrainGridLayer, nav2_costmap_2d::Layer)
