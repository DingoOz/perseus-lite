#pragma once

/// @file grid_geometry.hpp
/// @brief Geometry of a regular 2D grid laid over a world frame.

#include <cstddef>

namespace perseus_terrain
{
    /// @brief An axis-aligned grid of square cells in some world frame.
    /// @details Cell (0, 0) has its lower-left corner at (origin_x, origin_y). Cells are
    /// stored row-major with x varying fastest, which is the same layout as
    /// nav_msgs/OccupancyGrid, so a grid can be copied into a message without reordering.
    struct GridGeometry
    {
        double origin_x{0.0};
        double origin_y{0.0};
        double resolution{0.05};
        int width{0};
        int height{0};

        /// @brief Number of cells.
        std::size_t size() const;

        /// @brief Whether a cell index lies inside the grid.
        bool contains(int cx, int cy) const;

        /// @brief Row-major index of a cell. The cell must be inside the grid.
        std::size_t index(int cx, int cy) const;

        /// @brief Cell containing a world point.
        /// @return False if the point is outside the grid (cx, cy are still written).
        bool world_to_cell(double x, double y, int& cx, int& cy) const;

        /// @brief World coordinates of a cell's centre.
        void cell_center(int cx, int cy, double& x, double& y) const;

        /// @brief World x of the far (max) edge of the grid.
        double max_x() const;
        /// @brief World y of the far (max) edge of the grid.
        double max_y() const;
    };

    /// @brief A square window centred (to within one cell) on (x, y).
    /// @details The origin is snapped to a multiple of @p resolution, so every window
    /// built this way shares one global lattice. Without the snap, a window that follows
    /// the robot would re-bin the same ground differently every cycle and flicker.
    /// @param x Window centre in the world frame.
    /// @param y Window centre in the world frame.
    /// @param half_extent Half the side length, in metres.
    /// @param resolution Cell size, in metres.
    GridGeometry centred_window(double x, double y, double half_extent, double resolution);

}
