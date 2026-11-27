#include "perseus_terrain/log_odds_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace perseus_terrain
{
    namespace
    {
        constexpr float NEVER_OBSERVED = std::numeric_limits<float>::quiet_NaN();

        /// @brief Whole cells between two lattice-aligned origins.
        int cell_offset(double from, double to, double resolution)
        {
            return static_cast<int>(std::lround((to - from) / resolution));
        }
    }

    LogOddsMap::LogOddsMap(double resolution, MemoryParams params)
        : resolution_(resolution),
          params_(params)
    {
        geometry_.resolution = resolution;
    }

    bool LogOddsMap::empty() const
    {
        return geometry_.size() == 0;
    }

    const GridGeometry& LogOddsMap::geometry() const
    {
        return geometry_;
    }

    const std::vector<float>& LogOddsMap::log_odds() const
    {
        return log_odds_;
    }

    void LogOddsMap::ensure_contains(double min_x, double min_y, double max_x, double max_y)
    {
        if (empty())
        {
            const double center_x = 0.5 * (min_x + max_x);
            const double center_y = 0.5 * (min_y + max_y);
            const double half = 0.5 * std::max({params_.initial_size_m, max_x - min_x, max_y - min_y});
            geometry_ = centred_window(center_x, center_y, half, resolution_);
            log_odds_.assign(geometry_.size(), NEVER_OBSERVED);
            roughness_.assign(geometry_.size(), cost::FREE);
            return;
        }

        const double step = std::max(params_.growth_step_m, resolution_);
        auto steps_needed = [step](double overshoot)
        {
            return overshoot > 0.0 ? std::ceil(overshoot / step) * step : 0.0;
        };
        const double grow_left = steps_needed(geometry_.origin_x - min_x);
        const double grow_down = steps_needed(geometry_.origin_y - min_y);
        const double grow_right = steps_needed(max_x - geometry_.max_x());
        const double grow_up = steps_needed(max_y - geometry_.max_y());
        if (grow_left == 0.0 && grow_down == 0.0 && grow_right == 0.0 && grow_up == 0.0)
        {
            return;
        }

        GridGeometry grown = geometry_;
        grown.origin_x -= grow_left;
        grown.origin_y -= grow_down;
        // Snap to the lattice again: whole growth steps are whole cells only when the
        // step is a multiple of the resolution.
        grown.origin_x = std::floor(grown.origin_x / resolution_ + 0.5) * resolution_;
        grown.origin_y = std::floor(grown.origin_y / resolution_ + 0.5) * resolution_;
        grown.width = static_cast<int>(std::lround((geometry_.max_x() + grow_right - grown.origin_x) / resolution_));
        grown.height = static_cast<int>(std::lround((geometry_.max_y() + grow_up - grown.origin_y) / resolution_));
        const double max_cells = params_.max_size_m / resolution_;
        if (static_cast<double>(grown.width) > max_cells || static_cast<double>(grown.height) > max_cells)
        {
            return;
        }

        std::vector<float> log_odds(grown.size(), NEVER_OBSERVED);
        std::vector<std::int8_t> roughness(grown.size(), cost::FREE);
        const int shift_x = cell_offset(grown.origin_x, geometry_.origin_x, resolution_);
        const int shift_y = cell_offset(grown.origin_y, geometry_.origin_y, resolution_);
        for (int cy = 0; cy < geometry_.height; ++cy)
        {
            for (int cx = 0; cx < geometry_.width; ++cx)
            {
                const std::size_t from = geometry_.index(cx, cy);
                const std::size_t to = grown.index(cx + shift_x, cy + shift_y);
                log_odds[to] = log_odds_[from];
                roughness[to] = roughness_[from];
            }
        }
        geometry_ = grown;
        log_odds_ = std::move(log_odds);
        roughness_ = std::move(roughness);
    }

    bool LogOddsMap::fold(const TerrainObservation& observation)
    {
        const GridGeometry& source = observation.geometry;
        if (source.size() == 0 || observation.cost.size() != source.size() ||
            std::abs(source.resolution - resolution_) > 1e-9)
        {
            return false;
        }
        ensure_contains(source.origin_x, source.origin_y, source.max_x(), source.max_y());
        if (empty())
        {
            return false;
        }

        const int shift_x = cell_offset(geometry_.origin_x, source.origin_x, resolution_);
        const int shift_y = cell_offset(geometry_.origin_y, source.origin_y, resolution_);
        bool any = false;
        for (int cy = 0; cy < source.height; ++cy)
        {
            for (int cx = 0; cx < source.width; ++cx)
            {
                const std::int8_t verdict = observation.cost[source.index(cx, cy)];
                const int mx = cx + shift_x;
                const int my = cy + shift_y;
                if (verdict == cost::UNKNOWN || !geometry_.contains(mx, my))
                {
                    continue;
                }
                const std::size_t cell = geometry_.index(mx, my);
                float& value = log_odds_[cell];
                const double previous = std::isnan(value) ? 0.0 : static_cast<double>(value);
                const double delta = verdict == cost::OBSTACLE ? params_.log_odds_hit : -params_.log_odds_miss;
                value = static_cast<float>(std::clamp(previous + delta, params_.log_odds_min, params_.log_odds_max));
                if (verdict != cost::OBSTACLE)
                {
                    roughness_[cell] = verdict;
                }
                any = true;
            }
        }
        return any;
    }

    void LogOddsMap::forget_disc(double x, double y, double radius)
    {
        if (empty() || radius <= 0.0)
        {
            return;
        }
        const double radius_squared = radius * radius;
        int min_cx = 0;
        int min_cy = 0;
        int max_cx = 0;
        int max_cy = 0;
        geometry_.world_to_cell(x - radius, y - radius, min_cx, min_cy);
        geometry_.world_to_cell(x + radius, y + radius, max_cx, max_cy);
        for (int cy = std::max(0, min_cy); cy <= std::min(geometry_.height - 1, max_cy); ++cy)
        {
            for (int cx = std::max(0, min_cx); cx <= std::min(geometry_.width - 1, max_cx); ++cx)
            {
                double center_x = 0.0;
                double center_y = 0.0;
                geometry_.cell_center(cx, cy, center_x, center_y);
                const double dx = center_x - x;
                const double dy = center_y - y;
                if (dx * dx + dy * dy <= radius_squared)
                {
                    const std::size_t cell = geometry_.index(cx, cy);
                    log_odds_[cell] = NEVER_OBSERVED;
                    roughness_[cell] = cost::FREE;
                }
            }
        }
    }

    void LogOddsMap::mark_free(const Polygon& polygon)
    {
        if (empty() || polygon.size() < 3)
        {
            return;
        }
        double min_x = std::numeric_limits<double>::max();
        double min_y = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double max_y = std::numeric_limits<double>::lowest();
        for (const auto& [px, py] : polygon)
        {
            min_x = std::min(min_x, px);
            min_y = std::min(min_y, py);
            max_x = std::max(max_x, px);
            max_y = std::max(max_y, py);
        }
        int min_cx = 0;
        int min_cy = 0;
        int max_cx = 0;
        int max_cy = 0;
        geometry_.world_to_cell(min_x, min_y, min_cx, min_cy);
        geometry_.world_to_cell(max_x, max_y, max_cx, max_cy);
        for (int cy = std::max(0, min_cy); cy <= std::min(geometry_.height - 1, max_cy); ++cy)
        {
            for (int cx = std::max(0, min_cx); cx <= std::min(geometry_.width - 1, max_cx); ++cx)
            {
                double center_x = 0.0;
                double center_y = 0.0;
                geometry_.cell_center(cx, cy, center_x, center_y);
                if (polygon_contains(polygon, center_x, center_y))
                {
                    const std::size_t cell = geometry_.index(cx, cy);
                    log_odds_[cell] = static_cast<float>(params_.log_odds_min);
                    roughness_[cell] = cost::FREE;
                }
            }
        }
    }

    void LogOddsMap::reset()
    {
        std::fill(log_odds_.begin(), log_odds_.end(), NEVER_OBSERVED);
        std::fill(roughness_.begin(), roughness_.end(), cost::FREE);
    }

    std::vector<std::int8_t> LogOddsMap::to_cost() const
    {
        std::vector<std::int8_t> out(log_odds_.size(), cost::UNKNOWN);
        for (std::size_t cell = 0; cell < log_odds_.size(); ++cell)
        {
            const float value = log_odds_[cell];
            if (std::isnan(value))
            {
                continue;
            }
            out[cell] = static_cast<double>(value) > params_.occupied_threshold ? cost::OBSTACLE : roughness_[cell];
        }
        return out;
    }

}
