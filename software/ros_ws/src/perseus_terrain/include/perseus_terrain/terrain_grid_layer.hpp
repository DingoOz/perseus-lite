#pragma once

/// @file terrain_grid_layer.hpp
/// @brief A Nav2 costmap layer that copies a terrain OccupancyGrid into the costmap.
///
/// Unlike nav2's StaticLayer it works whatever frame the grid is in: each costmap cell
/// is looked up through TF, so the odom-frame terrain memory drops straight into the
/// map-frame global costmap and moves with every SLAM correction. It also takes the
/// worst terrain cell under each costmap cell (never losing a rock smaller than a
/// costmap cell), and it can ignore a grid that has gone stale.
///
/// Parameters (all under the layer's name):
///   enabled           bool    true
///   topic             string  /terrain/global_grid
///   transient_local   bool    true   Must match the publisher: true for the latched
///                                    memory, false for the volatile local grid.
///   max_age_s         double  0.0    Ignore a grid older than this (0 = never). Set it
///                                    for the local grid so a dead camera stops counting.
///   lethal_threshold  int     100    Grid values at or above this are lethal.
///
/// Clearing the costmap does not clear terrain memory (the next update would copy it
/// straight back). Call terrain_mapper's ~/clear_around_robot service for that; the
/// lite terrain behaviour tree does both.

#include <mutex>
#include <nav2_costmap_2d/layer.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "perseus_terrain/grid_geometry.hpp"

namespace perseus_terrain
{
    /// @brief See the file comment.
    class TerrainGridLayer : public nav2_costmap_2d::Layer
    {
    public:
        TerrainGridLayer() = default;

        void onInitialize() override;
        void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                          double* max_y) override;
        void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;
        void reset() override;
        bool isClearable() override;

    private:
        /// @brief Planar transform from the costmap's frame into the grid's cell frame
        ///        (origin at cell (0, 0)'s corner, axes along the grid).
        struct PlanarTransform
        {
            double cos_yaw;
            double sin_yaw;
            double tx;
            double ty;
        };

        void on_grid(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& message);
        bool grid_is_usable();

        std::mutex mutex_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
        std::string topic_;
        bool transient_local_{true};
        double max_age_s_{0.0};
        int lethal_threshold_{100};

        bool have_grid_{false};
        std::string grid_frame_;
        rclcpp::Time received_;
        double grid_origin_x_{0.0};
        double grid_origin_y_{0.0};
        double grid_origin_yaw_{0.0};
        GridGeometry geometry_;
        std::vector<std::int8_t> data_;

        std::optional<PlanarTransform> costmap_to_grid_;
        bool have_applied_bounds_{false};
        double applied_min_x_{0.0};
        double applied_min_y_{0.0};
        double applied_max_x_{0.0};
        double applied_max_y_{0.0};
    };

}
