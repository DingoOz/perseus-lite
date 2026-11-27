#include "perseus_terrain/grid_geometry.hpp"

#include <cmath>

namespace perseus_terrain
{
    std::size_t GridGeometry::size() const
    {
        if (width <= 0 || height <= 0)
        {
            return 0;
        }
        return static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    }

    bool GridGeometry::contains(int cx, int cy) const
    {
        return cx >= 0 && cy >= 0 && cx < width && cy < height;
    }

    std::size_t GridGeometry::index(int cx, int cy) const
    {
        return static_cast<std::size_t>(cy) * static_cast<std::size_t>(width) + static_cast<std::size_t>(cx);
    }

    bool GridGeometry::world_to_cell(double x, double y, int& cx, int& cy) const
    {
        cx = static_cast<int>(std::floor((x - origin_x) / resolution));
        cy = static_cast<int>(std::floor((y - origin_y) / resolution));
        return contains(cx, cy);
    }

    void GridGeometry::cell_center(int cx, int cy, double& x, double& y) const
    {
        x = origin_x + (static_cast<double>(cx) + 0.5) * resolution;
        y = origin_y + (static_cast<double>(cy) + 0.5) * resolution;
    }

    double GridGeometry::max_x() const
    {
        return origin_x + static_cast<double>(width) * resolution;
    }

    double GridGeometry::max_y() const
    {
        return origin_y + static_cast<double>(height) * resolution;
    }

    GridGeometry centred_window(double x, double y, double half_extent, double resolution)
    {
        GridGeometry geometry;
        geometry.resolution = resolution;
        const int half_cells = static_cast<int>(std::ceil(half_extent / resolution));
        geometry.width = 2 * half_cells;
        geometry.height = 2 * half_cells;
        geometry.origin_x = (std::floor(x / resolution) - static_cast<double>(half_cells)) * resolution;
        geometry.origin_y = (std::floor(y / resolution) - static_cast<double>(half_cells)) * resolution;
        return geometry;
    }

}
