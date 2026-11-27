#pragma once

/// @file terrain_classifier.hpp
/// @brief Height-difference terrain classification of a short window of 3D points.
///
/// The idea, in one paragraph: bin the points into cells and take a robust "floor"
/// height for each. If the ground may slope by at most max_slope_deg, then a
/// neighbour d metres away whose floor is at z_n says this cell's ground cannot be
/// higher than z_n + d * tan(max_slope). The lowest of those bounds over a small
/// window is this cell's ground estimate. A floor that sits well above that estimate
/// is a step up (a rock face); a floor well below the highest lower bound is a step
/// down (a hole or edge); a tall return above the ground is something standing in the
/// cell. None of this needs a plane fit or a flat-world assumption, so a uniform slope,
/// or a robot that is itself tilted on a dune, reads as drivable.
///
/// It is only valid on points measured close together in time: it compares heights
/// between cells, so odometry drift between two passes over the same ground would show
/// up as a false step. Feed it a buffer of the last second or so, never a long-term
/// accumulated cloud (that is what LogOddsMap is for).

#include <cstdint>
#include <vector>

#include "perseus_terrain/grid_geometry.hpp"

namespace perseus_terrain
{
    /// @brief A 3D point in the grid's world frame.
    struct Point3
    {
        float x{0.0F};
        float y{0.0F};
        float z{0.0F};
    };

    /// @brief nav_msgs/OccupancyGrid-style cell values used throughout this package.
    namespace cost
    {
        /// @brief Never observed, or too few points to judge.
        constexpr std::int8_t UNKNOWN = -1;
        /// @brief Observed and flat.
        constexpr std::int8_t FREE = 0;
        /// @brief Observed and not drivable.
        constexpr std::int8_t OBSTACLE = 100;
    }

    /// @brief Thresholds of the classifier. Defaults suit the 50 %-scale lite chassis on
    ///        sand, with a stereo depth camera.
    struct ClassifierParams
    {
        /// @brief Half-width of the neighbourhood the ground estimate looks across, in
        ///        metres. Larger windows judge longer slopes but grow every step-down
        ///        detection outwards by up to this distance.
        double ground_window_m{0.2};
        /// @brief Steepest slope still treated as ground, in degrees.
        double max_slope_deg{25.0};
        /// @brief Floor rising more than this above the ground estimate is an obstacle.
        double max_step_up_m{0.06};
        /// @brief Floor dropping more than this below its surroundings is an obstacle.
        ///        Zero disables the test.
        double max_step_down_m{0.08};
        /// @brief A return this far above the ground, inside a cell, is something
        ///        standing there (a post, a pipe on its side).
        double max_height_above_ground_m{0.06};
        /// @brief Returns more than this above the ground are ignored entirely, so the
        ///        robot may drive under them. Zero disables the cap.
        double overhead_clearance_m{0.45};
        /// @brief Cells with fewer points are unknown.
        int min_points_per_cell{3};
        /// @brief Obstacle blobs (8-connected) smaller than this are erased as noise.
        int min_obstacle_cells{2};
        /// @brief Percentile (0..1) of a cell's z values used as its floor height. 0 is the
        ///        minimum, which is right for a LiDAR; stereo depth noise makes the
        ///        minimum an outlier, so the default is a low percentile instead.
        double floor_percentile{0.1};
        /// @brief Percentile (0..1) of a cell's z values used as its top height.
        double top_percentile{0.9};
        /// @brief Fraction (0..1) of the nearest obstacle threshold at which a cell
        ///        starts to cost something. Below it a cell is FREE.
        double rough_start_fraction{0.5};
        /// @brief Cost (1..99) of a cell that is just under an obstacle threshold.
        int rough_max_cost{60};
    };

    /// @brief Result of classify_terrain(): a cost grid plus the intermediate layers,
    ///        which are worth publishing when tuning.
    struct TerrainObservation
    {
        GridGeometry geometry;
        /// @brief UNKNOWN, FREE, a roughness cost (1..rough_max_cost), or OBSTACLE.
        std::vector<std::int8_t> cost;
        /// @brief Robust floor height per cell; NaN where unknown.
        std::vector<float> floor;
        /// @brief Ground estimate per cell; NaN where unknown.
        std::vector<float> ground;
        /// @brief Floor above ground, metres (>= 0); NaN where unknown.
        std::vector<float> step_up;
        /// @brief Surroundings above floor, metres (>= 0); NaN where unknown.
        std::vector<float> step_down;
        /// @brief Top of the cell's returns above ground, metres; NaN where unknown.
        std::vector<float> height_above_ground;
        /// @brief Points that fell in each cell.
        std::vector<std::uint32_t> point_count;
    };

    /// @brief Classifies every cell of @p geometry from @p points.
    /// @param points Points in the grid's world frame. Points outside the grid are ignored.
    /// @param geometry The grid to fill.
    /// @param params Classifier thresholds.
    TerrainObservation classify_terrain(const std::vector<Point3>& points, const GridGeometry& geometry,
                                        const ClassifierParams& params);

    /// @brief Sets OBSTACLE cells that belong to an 8-connected blob of fewer than
    ///        @p min_cells cells to FREE.
    void remove_small_obstacles(std::vector<std::int8_t>& cost, const GridGeometry& geometry, int min_cells);

}
