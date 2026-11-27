#include "perseus_terrain/footprint.hpp"

#include <cmath>

namespace perseus_terrain
{
    Polygon polygon_from_flat(const std::vector<double>& flat)
    {
        Polygon polygon;
        if (flat.size() % 2 != 0 || flat.size() < 6)
        {
            return polygon;
        }
        for (std::size_t i = 0; i < flat.size(); i += 2)
        {
            polygon.emplace_back(flat[i], flat[i + 1]);
        }
        return polygon;
    }

    Polygon pad_polygon(const Polygon& polygon, double padding)
    {
        if (polygon.empty() || padding == 0.0)
        {
            return polygon;
        }
        double center_x = 0.0;
        double center_y = 0.0;
        for (const auto& [x, y] : polygon)
        {
            center_x += x;
            center_y += y;
        }
        center_x /= static_cast<double>(polygon.size());
        center_y /= static_cast<double>(polygon.size());
        Polygon padded;
        padded.reserve(polygon.size());
        for (const auto& [x, y] : polygon)
        {
            const double dx = x - center_x;
            const double dy = y - center_y;
            const double length = std::hypot(dx, dy);
            if (length < 1e-9)
            {
                padded.emplace_back(x, y);
                continue;
            }
            const double scale = (length + padding) / length;
            padded.emplace_back(center_x + dx * scale, center_y + dy * scale);
        }
        return padded;
    }

    Polygon transform_polygon(const Polygon& polygon, double x, double y, double yaw)
    {
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        Polygon placed;
        placed.reserve(polygon.size());
        for (const auto& [px, py] : polygon)
        {
            placed.emplace_back(x + c * px - s * py, y + s * px + c * py);
        }
        return placed;
    }

    bool polygon_contains(const Polygon& polygon, double x, double y)
    {
        bool inside = false;
        const std::size_t count = polygon.size();
        for (std::size_t i = 0, j = count - 1; i < count; j = i++)
        {
            const auto& [xi, yi] = polygon[i];
            const auto& [xj, yj] = polygon[j];
            if (((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi))
            {
                inside = !inside;
            }
        }
        return inside;
    }

}
