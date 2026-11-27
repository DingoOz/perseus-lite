#pragma once

/// @file log_odds_map.hpp
/// @brief Long-term terrain memory: per-cell log-odds of "obstacle", folded from a
///        stream of short-window TerrainObservations.
///
/// Classification happens on a short, drift-free window; only the verdicts are
/// remembered. A rock seen once stays on the map after it leaves the camera's narrow
/// field of view, and a cell that was wrongly marked (dust, a speckle) clears again the
/// next time the camera looks at it and sees flat ground.

#include <cstdint>
#include <vector>

#include "perseus_terrain/footprint.hpp"
#include "perseus_terrain/grid_geometry.hpp"
#include "perseus_terrain/terrain_classifier.hpp"

namespace perseus_terrain
{
    /// @brief Evidence weights and growth policy of the memory.
    struct MemoryParams
    {
        /// @brief Added to a cell's log-odds each time it is observed as an obstacle.
        ///        Above occupied_threshold on its own, so one sighting is enough.
        double log_odds_hit{0.7};
        /// @brief Subtracted each time a cell is observed as drivable.
        double log_odds_miss{0.4};
        /// @brief Clamp: how much "free" evidence a cell can bank.
        double log_odds_min{-2.0};
        /// @brief Clamp: how much "obstacle" evidence a cell can bank. Max / miss is the
        ///        number of clear sightings it takes to forget a well-established rock.
        double log_odds_max{3.5};
        /// @brief Log-odds above which a cell is reported as an obstacle.
        double occupied_threshold{0.5};
        /// @brief Side of the first allocation, in metres, centred on the first observation.
        double initial_size_m{20.0};
        /// @brief The grid grows by whole multiples of this, in metres, when an observation
        ///        reaches past its edge.
        double growth_step_m{5.0};
        /// @brief Hard limit on either side length, in metres. Observations beyond it are
        ///        dropped rather than growing the map without bound.
        double max_size_m{200.0};
    };

    /// @brief See the file comment.
    class LogOddsMap
    {
    public:
        LogOddsMap(double resolution, MemoryParams params);

        /// @brief Whether anything has been allocated yet.
        bool empty() const;

        const GridGeometry& geometry() const;

        /// @brief Folds one observation in. Its geometry must share this map's lattice
        ///        (same resolution, origin a whole number of cells away), which is true
        ///        of anything built with centred_window().
        /// @return False if the observation was incompatible or wholly out of range.
        bool fold(const TerrainObservation& observation);

        /// @brief Forgets every cell whose centre is within @p radius of (x, y).
        void forget_disc(double x, double y, double radius);

        /// @brief Marks every cell whose centre is inside @p polygon as observed-free.
        /// @details Used for the cells under the robot: nothing can be under it, and the
        /// camera cannot see there to clear a stale verdict.
        void mark_free(const Polygon& polygon);

        /// @brief Forgets everything (keeps the allocation).
        void reset();

        /// @brief The memory as OccupancyGrid values: UNKNOWN, a remembered roughness
        ///        cost, or OBSTACLE.
        std::vector<std::int8_t> to_cost() const;

        /// @brief Raw log-odds per cell; NaN where never observed. For tests and debugging.
        const std::vector<float>& log_odds() const;

    private:
        /// @brief Grows the allocation so the rectangle fits, within max_size_m.
        void ensure_contains(double min_x, double min_y, double max_x, double max_y);

        double resolution_;
        MemoryParams params_;
        GridGeometry geometry_;
        std::vector<float> log_odds_;
        std::vector<std::int8_t> roughness_;
    };

}
