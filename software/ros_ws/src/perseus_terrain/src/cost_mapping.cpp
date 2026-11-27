#include "perseus_terrain/cost_mapping.hpp"

#include <algorithm>
#include <cmath>

namespace perseus_terrain
{
    std::uint8_t occupancy_to_cost(std::int8_t value, int lethal_threshold)
    {
        if (value < 0)
        {
            return costmap_value::NO_INFORMATION;
        }
        const int threshold = std::clamp(lethal_threshold, 1, 100);
        if (value >= threshold)
        {
            return costmap_value::LETHAL_OBSTACLE;
        }
        if (threshold == 1)
        {
            return costmap_value::FREE_SPACE;
        }
        const double scaled = static_cast<double>(value) * costmap_value::MAX_NON_OBSTACLE / static_cast<double>(threshold - 1);
        return static_cast<std::uint8_t>(std::clamp(std::lround(scaled), 0L, static_cast<long>(costmap_value::MAX_NON_OBSTACLE)));
    }

    std::int8_t sample_worst(const GridGeometry& geometry, const std::vector<std::int8_t>& data, double x, double y,
                             double size)
    {
        if (data.size() != geometry.size() || geometry.size() == 0)
        {
            return -1;
        }
        const double half = 0.5 * size;
        // Shrink by a hair so a square exactly one cell wide, placed exactly on that
        // cell, covers one cell and not its neighbours too.
        const double epsilon = 1e-6 * geometry.resolution;
        int min_cx = 0;
        int min_cy = 0;
        int max_cx = 0;
        int max_cy = 0;
        geometry.world_to_cell(x - half + epsilon, y - half + epsilon, min_cx, min_cy);
        geometry.world_to_cell(x + half - epsilon, y + half - epsilon, max_cx, max_cy);
        min_cx = std::max(min_cx, 0);
        min_cy = std::max(min_cy, 0);
        max_cx = std::min(max_cx, geometry.width - 1);
        max_cy = std::min(max_cy, geometry.height - 1);
        std::int8_t worst = -1;
        for (int cy = min_cy; cy <= max_cy; ++cy)
        {
            for (int cx = min_cx; cx <= max_cx; ++cx)
            {
                worst = std::max(worst, data[geometry.index(cx, cy)]);
            }
        }
        return worst;
    }

    std::uint8_t combine_max(std::uint8_t existing, std::uint8_t incoming)
    {
        if (incoming == costmap_value::NO_INFORMATION)
        {
            return existing;
        }
        if (existing == costmap_value::NO_INFORMATION)
        {
            return incoming;
        }
        return std::max(existing, incoming);
    }

}
