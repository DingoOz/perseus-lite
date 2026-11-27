#pragma once

/// @file footprint.hpp
/// @brief Robot footprint polygons.

#include <string>
#include <utility>
#include <vector>

namespace perseus_terrain
{
    /// @brief A polygon as (x, y) vertices in order.
    using Polygon = std::vector<std::pair<double, double>>;

    /// @brief Builds a polygon from a flat [x0, y0, x1, y1, ...] list.
    /// @return An empty polygon if the list has an odd length or fewer than 3 vertices.
    Polygon polygon_from_flat(const std::vector<double>& flat);

    /// @brief Grows (or shrinks, if negative) a polygon by moving every vertex
    ///        @p padding further from the centroid along the line through it.
    Polygon pad_polygon(const Polygon& polygon, double padding);

    /// @brief Places a robot-frame polygon at pose (x, y, yaw) in the world.
    Polygon transform_polygon(const Polygon& polygon, double x, double y, double yaw);

    /// @brief Even-odd point-in-polygon test.
    bool polygon_contains(const Polygon& polygon, double x, double y);

}
