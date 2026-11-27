#include "perseus_terrain/terrain_classifier.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace perseus_terrain
{
    namespace
    {
        constexpr float NOT_A_NUMBER = std::numeric_limits<float>::quiet_NaN();

        /// @brief The @p percentile (0..1) of @p values, partially reordering them.
        float percentile_of(std::vector<float>::iterator begin, std::vector<float>::iterator end, double percentile)
        {
            const auto count = std::distance(begin, end);
            const double clamped = std::clamp(percentile, 0.0, 1.0);
            const auto rank = static_cast<std::ptrdiff_t>(std::lround(clamped * static_cast<double>(count - 1)));
            std::nth_element(begin, begin + rank, end);
            return *(begin + rank);
        }

        /// @brief One neighbour offset of the ground window and the height the slope
        ///        allows across it.
        struct WindowOffset
        {
            int dx;
            int dy;
            float slack;
        };

        std::vector<WindowOffset> build_window(const GridGeometry& geometry, const ClassifierParams& params)
        {
            const int radius = std::max(1, static_cast<int>(std::lround(params.ground_window_m / geometry.resolution)));
            const double slope_per_metre = std::tan(params.max_slope_deg * M_PI / 180.0);
            std::vector<WindowOffset> window;
            for (int dy = -radius; dy <= radius; ++dy)
            {
                for (int dx = -radius; dx <= radius; ++dx)
                {
                    const double distance_cells = std::hypot(static_cast<double>(dx), static_cast<double>(dy));
                    // A disc, not a square: the corners of a square window would look
                    // ~41 % further along the diagonals than along the axes.
                    if (distance_cells > static_cast<double>(radius) + 1e-9)
                    {
                        continue;
                    }
                    window.push_back({dx, dy, static_cast<float>(distance_cells * geometry.resolution * slope_per_metre)});
                }
            }
            return window;
        }

        /// @brief Fraction of a threshold a value reaches; 0 when the test is disabled.
        double fraction_of(float value, double threshold)
        {
            if (threshold <= 0.0 || std::isnan(value))
            {
                return 0.0;
            }
            return static_cast<double>(value) / threshold;
        }
    }

    TerrainObservation classify_terrain(const std::vector<Point3>& points, const GridGeometry& geometry,
                                        const ClassifierParams& params)
    {
        const std::size_t cells = geometry.size();
        TerrainObservation observation;
        observation.geometry = geometry;
        observation.cost.assign(cells, cost::UNKNOWN);
        observation.floor.assign(cells, NOT_A_NUMBER);
        observation.ground.assign(cells, NOT_A_NUMBER);
        observation.step_up.assign(cells, NOT_A_NUMBER);
        observation.step_down.assign(cells, NOT_A_NUMBER);
        observation.height_above_ground.assign(cells, NOT_A_NUMBER);
        observation.point_count.assign(cells, 0U);
        if (cells == 0)
        {
            return observation;
        }

        // 1. Bucket the z values by cell (a counting sort), so each cell's heights are
        //    contiguous and a percentile is one nth_element away.
        constexpr std::size_t OUTSIDE = std::numeric_limits<std::size_t>::max();
        std::vector<std::size_t> cell_of_point(points.size(), OUTSIDE);
        for (std::size_t i = 0; i < points.size(); ++i)
        {
            const Point3& point = points[i];
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            {
                continue;
            }
            int cx = 0;
            int cy = 0;
            if (geometry.world_to_cell(point.x, point.y, cx, cy))
            {
                const std::size_t cell = geometry.index(cx, cy);
                cell_of_point[i] = cell;
                ++observation.point_count[cell];
            }
        }
        std::vector<std::size_t> offsets(cells + 1, 0);
        for (std::size_t cell = 0; cell < cells; ++cell)
        {
            offsets[cell + 1] = offsets[cell] + observation.point_count[cell];
        }
        std::vector<float> heights(offsets[cells]);
        {
            std::vector<std::size_t> cursor(offsets.begin(), offsets.end() - 1);
            for (std::size_t i = 0; i < points.size(); ++i)
            {
                if (cell_of_point[i] != OUTSIDE)
                {
                    heights[cursor[cell_of_point[i]]++] = points[i].z;
                }
            }
        }

        // 2. A robust floor per cell. Sparse cells stay NaN, so a lone speckle point can
        //    never become a neighbour's reference height.
        const auto min_points = static_cast<std::uint32_t>(std::max(1, params.min_points_per_cell));
        for (std::size_t cell = 0; cell < cells; ++cell)
        {
            if (observation.point_count[cell] < min_points)
            {
                continue;
            }
            const auto begin = heights.begin() + static_cast<std::ptrdiff_t>(offsets[cell]);
            const auto end = heights.begin() + static_cast<std::ptrdiff_t>(offsets[cell + 1]);
            observation.floor[cell] = percentile_of(begin, end, params.floor_percentile);
        }

        // 3. Ground estimate and the two step tests, from the slope-allowance window.
        //    The window contains the cell itself (slack 0), so the ground is never above
        //    the floor and the lower bound never below it: both steps are >= 0.
        const std::vector<WindowOffset> window = build_window(geometry, params);
        for (int cy = 0; cy < geometry.height; ++cy)
        {
            for (int cx = 0; cx < geometry.width; ++cx)
            {
                const std::size_t cell = geometry.index(cx, cy);
                const float floor = observation.floor[cell];
                if (std::isnan(floor))
                {
                    continue;
                }
                float upper = std::numeric_limits<float>::max();
                float lower = std::numeric_limits<float>::lowest();
                for (const WindowOffset& offset : window)
                {
                    const int nx = cx + offset.dx;
                    const int ny = cy + offset.dy;
                    if (!geometry.contains(nx, ny))
                    {
                        continue;
                    }
                    const float neighbour = observation.floor[geometry.index(nx, ny)];
                    if (std::isnan(neighbour))
                    {
                        continue;
                    }
                    upper = std::min(upper, neighbour + offset.slack);
                    lower = std::max(lower, neighbour - offset.slack);
                }
                observation.ground[cell] = upper;
                observation.step_up[cell] = floor - upper;
                observation.step_down[cell] = lower - floor;
            }
        }

        // 4. The tallest structure in each cell, measured from the ground estimate and
        //    ignoring anything high enough to drive under.
        std::vector<float> below_cap;
        for (std::size_t cell = 0; cell < cells; ++cell)
        {
            const float ground = observation.ground[cell];
            if (std::isnan(ground))
            {
                continue;
            }
            below_cap.clear();
            for (std::size_t i = offsets[cell]; i < offsets[cell + 1]; ++i)
            {
                const float above = heights[i] - ground;
                if (params.overhead_clearance_m <= 0.0 || above <= static_cast<float>(params.overhead_clearance_m))
                {
                    below_cap.push_back(above);
                }
            }
            observation.height_above_ground[cell] =
                below_cap.empty() ? 0.0F : percentile_of(below_cap.begin(), below_cap.end(), params.top_percentile);
        }

        // 5. Verdict per cell: obstacle past any threshold, otherwise a soft cost that
        //    rises as the cell approaches one. The soft cost is what makes the planner
        //    prefer flat sand over a lumpy route of the same length.
        const double rough_span = 1.0 - params.rough_start_fraction;
        for (std::size_t cell = 0; cell < cells; ++cell)
        {
            if (std::isnan(observation.ground[cell]))
            {
                continue;
            }
            const double worst = std::max({fraction_of(observation.step_up[cell], params.max_step_up_m),
                                           fraction_of(observation.step_down[cell], params.max_step_down_m),
                                           fraction_of(observation.height_above_ground[cell], params.max_height_above_ground_m)});
            if (worst > 1.0)
            {
                observation.cost[cell] = cost::OBSTACLE;
            }
            else if (rough_span > 0.0 && params.rough_max_cost > 0 && worst > params.rough_start_fraction)
            {
                const double scaled = (worst - params.rough_start_fraction) / rough_span * params.rough_max_cost;
                const long rough = std::clamp(std::lround(scaled), 1L, static_cast<long>(std::min(params.rough_max_cost, 99)));
                observation.cost[cell] = static_cast<std::int8_t>(rough);
            }
            else
            {
                observation.cost[cell] = cost::FREE;
            }
        }

        remove_small_obstacles(observation.cost, geometry, params.min_obstacle_cells);
        return observation;
    }

    void remove_small_obstacles(std::vector<std::int8_t>& cost, const GridGeometry& geometry, int min_cells)
    {
        if (min_cells <= 1 || cost.size() != geometry.size())
        {
            return;
        }
        std::vector<bool> visited(cost.size(), false);
        std::vector<std::pair<int, int>> stack;
        std::vector<std::size_t> blob;
        for (int cy = 0; cy < geometry.height; ++cy)
        {
            for (int cx = 0; cx < geometry.width; ++cx)
            {
                const std::size_t start = geometry.index(cx, cy);
                if (visited[start] || cost[start] != cost::OBSTACLE)
                {
                    continue;
                }
                // 8-connected flood fill: a sparsely seen rock can touch its other half
                // only at a corner, and splitting it would erase both halves.
                blob.clear();
                stack.assign(1, {cx, cy});
                visited[start] = true;
                while (!stack.empty())
                {
                    const auto [x, y] = stack.back();
                    stack.pop_back();
                    blob.push_back(geometry.index(x, y));
                    for (int dy = -1; dy <= 1; ++dy)
                    {
                        for (int dx = -1; dx <= 1; ++dx)
                        {
                            const int nx = x + dx;
                            const int ny = y + dy;
                            if (!geometry.contains(nx, ny))
                            {
                                continue;
                            }
                            const std::size_t neighbour = geometry.index(nx, ny);
                            if (visited[neighbour] || cost[neighbour] != cost::OBSTACLE)
                            {
                                continue;
                            }
                            visited[neighbour] = true;
                            stack.emplace_back(nx, ny);
                        }
                    }
                }
                if (blob.size() < static_cast<std::size_t>(min_cells))
                {
                    for (const std::size_t cell : blob)
                    {
                        cost[cell] = cost::FREE;
                    }
                }
            }
        }
    }

}
