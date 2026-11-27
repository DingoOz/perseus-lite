#pragma once

/// @file cost_mapping.hpp
/// @brief Conversions from terrain grids (OccupancyGrid values) to Nav2 costmap costs.
///        Kept free of Nav2 headers so they can be unit-tested on their own.

#include <cstdint>
#include <vector>

#include "perseus_terrain/grid_geometry.hpp"

namespace perseus_terrain
{
    /// @brief Nav2 costmap_2d cost values (mirrors nav2_costmap_2d/cost_values.hpp).
    namespace costmap_value
    {
        constexpr std::uint8_t FREE_SPACE = 0;
        constexpr std::uint8_t MAX_NON_OBSTACLE = 252;
        constexpr std::uint8_t LETHAL_OBSTACLE = 254;
        constexpr std::uint8_t NO_INFORMATION = 255;
    }

    /// @brief Maps one OccupancyGrid value to a costmap cost.
    /// @details Unknown (< 0) becomes NO_INFORMATION; anything at or above
    /// @p lethal_threshold is LETHAL_OBSTACLE; the rest scale linearly onto 0..252, so a
    /// terrain roughness cost becomes an ordinary "prefer to avoid" cost for the planner.
    std::uint8_t occupancy_to_cost(std::int8_t value, int lethal_threshold);

    /// @brief The worst known value of the grid cells overlapping a square of side
    ///        @p size centred on (x, y), in the grid's frame.
    /// @details A master costmap cell can be coarser than the terrain grid. Sampling only
    /// its centre would let a one-cell rock fall between samples; taking the worst cell
    /// it covers never loses an obstacle.
    /// @return The worst value, or -1 if none of the covered cells is known.
    std::int8_t sample_worst(const GridGeometry& geometry, const std::vector<std::int8_t>& data, double x, double y,
                             double size);

    /// @brief Combines a new cost into an existing master-grid cost, keeping the worse
    ///        one; NO_INFORMATION in the master is always replaced.
    std::uint8_t combine_max(std::uint8_t existing, std::uint8_t incoming);

}
