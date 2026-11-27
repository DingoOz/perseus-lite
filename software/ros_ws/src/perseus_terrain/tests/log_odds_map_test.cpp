#include "perseus_terrain/log_odds_map.hpp"

#include <gtest/gtest.h>

#include <cmath>

using perseus_terrain::centred_window;
using perseus_terrain::GridGeometry;
using perseus_terrain::LogOddsMap;
using perseus_terrain::MemoryParams;
using perseus_terrain::TerrainObservation;
namespace cost = perseus_terrain::cost;

namespace
{
    constexpr double RES = 0.05;

    /// @brief A 1 m window around (x, y) with every cell set to @p value.
    TerrainObservation uniform_observation(double x, double y, std::int8_t value)
    {
        TerrainObservation observation;
        observation.geometry = centred_window(x, y, 0.5, RES);
        observation.cost.assign(observation.geometry.size(), value);
        return observation;
    }

    std::int8_t memory_at(const LogOddsMap& map, double x, double y)
    {
        int cx = 0;
        int cy = 0;
        if (!map.geometry().world_to_cell(x, y, cx, cy))
        {
            return -2;
        }
        return map.to_cost()[map.geometry().index(cx, cy)];
    }
}

TEST(LogOddsMap, OneObstacleSightingIsEnough)
{
    LogOddsMap map(RES, MemoryParams{});
    ASSERT_TRUE(map.fold(uniform_observation(0.0, 0.0, cost::OBSTACLE)));
    EXPECT_EQ(memory_at(map, 0.0, 0.0), cost::OBSTACLE);
    // Never-observed cells stay unknown.
    EXPECT_EQ(memory_at(map, 5.0, 5.0), cost::UNKNOWN);
}

TEST(LogOddsMap, RepeatedFreeSightingsClearAnObstacle)
{
    MemoryParams params;
    LogOddsMap map(RES, params);
    for (int i = 0; i < 10; ++i)
    {
        map.fold(uniform_observation(0.0, 0.0, cost::OBSTACLE));
    }
    // Saturated at log_odds_max (3.5); each miss takes 0.4 off; occupied above 0.5.
    int sightings = 0;
    while (memory_at(map, 0.0, 0.0) == cost::OBSTACLE && sightings < 100)
    {
        map.fold(uniform_observation(0.0, 0.0, cost::FREE));
        ++sightings;
    }
    const int expected = static_cast<int>(std::ceil((params.log_odds_max - params.occupied_threshold) / params.log_odds_miss));
    EXPECT_EQ(sightings, expected);
    EXPECT_EQ(memory_at(map, 0.0, 0.0), cost::FREE);
}

TEST(LogOddsMap, UnknownObservationsChangeNothing)
{
    LogOddsMap map(RES, MemoryParams{});
    map.fold(uniform_observation(0.0, 0.0, cost::OBSTACLE));
    map.fold(uniform_observation(0.0, 0.0, cost::UNKNOWN));
    EXPECT_EQ(memory_at(map, 0.0, 0.0), cost::OBSTACLE);
}

TEST(LogOddsMap, RemembersRoughness)
{
    LogOddsMap map(RES, MemoryParams{});
    map.fold(uniform_observation(0.0, 0.0, 30));
    EXPECT_EQ(memory_at(map, 0.0, 0.0), 30);
}

TEST(LogOddsMap, GrowingKeepsCellsInPlace)
{
    MemoryParams params;
    params.initial_size_m = 4.0;
    params.growth_step_m = 2.0;
    LogOddsMap map(RES, params);
    map.fold(uniform_observation(0.0, 0.0, cost::OBSTACLE));
    const GridGeometry before = map.geometry();
    map.fold(uniform_observation(10.0, -7.0, cost::FREE));
    const GridGeometry after = map.geometry();
    EXPECT_GT(after.width, before.width);
    EXPECT_GT(after.height, before.height);
    EXPECT_EQ(memory_at(map, 0.0, 0.0), cost::OBSTACLE);
    EXPECT_EQ(memory_at(map, 10.0, -7.0), cost::FREE);
    // Still on the same lattice.
    EXPECT_NEAR(std::remainder(after.origin_x - before.origin_x, RES), 0.0, 1e-9);
}

TEST(LogOddsMap, RespectsMaximumSize)
{
    MemoryParams params;
    params.initial_size_m = 4.0;
    params.max_size_m = 10.0;
    LogOddsMap map(RES, params);
    map.fold(uniform_observation(0.0, 0.0, cost::FREE));
    EXPECT_FALSE(map.fold(uniform_observation(100.0, 0.0, cost::OBSTACLE)));
    EXPECT_LE(map.geometry().width * RES, 10.0 + 1e-9);
}

TEST(LogOddsMap, RejectsAnotherResolution)
{
    LogOddsMap map(RES, MemoryParams{});
    TerrainObservation observation;
    observation.geometry = centred_window(0.0, 0.0, 0.5, 0.1);
    observation.cost.assign(observation.geometry.size(), cost::OBSTACLE);
    EXPECT_FALSE(map.fold(observation));
}

TEST(LogOddsMap, ForgetDiscAndMarkFree)
{
    LogOddsMap map(RES, MemoryParams{});
    map.fold(uniform_observation(0.0, 0.0, cost::OBSTACLE));
    map.forget_disc(0.0, 0.0, 0.2);
    EXPECT_EQ(memory_at(map, 0.0, 0.0), cost::UNKNOWN);
    EXPECT_EQ(memory_at(map, 0.4, 0.4), cost::OBSTACLE);

    map.mark_free({{0.3, 0.3}, {0.45, 0.3}, {0.45, 0.45}, {0.3, 0.45}});
    EXPECT_EQ(memory_at(map, 0.4, 0.4), cost::FREE);
    EXPECT_EQ(memory_at(map, -0.4, -0.4), cost::OBSTACLE);

    map.reset();
    EXPECT_EQ(memory_at(map, -0.4, -0.4), cost::UNKNOWN);
}
