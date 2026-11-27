#include "perseus_terrain/terrain_classifier.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <random>
#include <vector>

using perseus_terrain::ClassifierParams;
using perseus_terrain::classify_terrain;
using perseus_terrain::GridGeometry;
using perseus_terrain::Point3;
using perseus_terrain::remove_small_obstacles;
using perseus_terrain::TerrainObservation;
namespace cost = perseus_terrain::cost;

namespace
{
    constexpr double DEG = M_PI / 180.0;

    /// @brief A 3 m x 3 m grid of 5 cm cells with its origin at (0, 0).
    GridGeometry test_grid()
    {
        GridGeometry geometry;
        geometry.resolution = 0.05;
        geometry.width = 60;
        geometry.height = 60;
        return geometry;
    }

    /// @brief Samples a height field densely (1 cm) over a rectangle.
    void add_surface(std::vector<Point3>& points, double x0, double x1, double y0, double y1,
                     const std::function<double(double, double)>& height, double spacing = 0.01)
    {
        for (double x = x0 + spacing / 2; x < x1; x += spacing)
        {
            for (double y = y0 + spacing / 2; y < y1; y += spacing)
            {
                points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(height(x, y))});
            }
        }
    }

    std::int8_t cost_at(const TerrainObservation& observation, double x, double y)
    {
        int cx = 0;
        int cy = 0;
        EXPECT_TRUE(observation.geometry.world_to_cell(x, y, cx, cy));
        return observation.cost[observation.geometry.index(cx, cy)];
    }

    int count_cost(const TerrainObservation& observation, std::int8_t value)
    {
        int count = 0;
        for (const auto c : observation.cost)
        {
            count += c == value ? 1 : 0;
        }
        return count;
    }
}

TEST(TerrainClassifier, FlatGroundIsFree)
{
    std::vector<Point3> points;
    add_surface(points, 0.0, 3.0, 0.0, 3.0, [](double, double)
                { return 0.0; });
    const auto observation = classify_terrain(points, test_grid(), ClassifierParams{});
    EXPECT_EQ(count_cost(observation, cost::FREE), 3600);
}

TEST(TerrainClassifier, TiltedRobotOnFlatGroundIsStillFree)
{
    // A uniform tilt (as when the robot itself pitches on a dune and the 2D EKF does
    // not know) must not read as an obstacle.
    std::vector<Point3> points;
    add_surface(points, 0.0, 3.0, 0.0, 3.0, [](double x, double y)
                { return std::tan(10 * DEG) * x + 0.05 * y; });
    const auto observation = classify_terrain(points, test_grid(), ClassifierParams{});
    EXPECT_EQ(count_cost(observation, cost::OBSTACLE), 0);
}

TEST(TerrainClassifier, SlopeBelowLimitIsDrivable)
{
    std::vector<Point3> points;
    add_surface(points, 0.0, 3.0, 0.0, 3.0, [](double x, double)
                { return std::tan(15 * DEG) * x; });
    const auto observation = classify_terrain(points, test_grid(), ClassifierParams{});
    EXPECT_EQ(count_cost(observation, cost::OBSTACLE), 0);
}

TEST(TerrainClassifier, SlopeAboveLimitIsObstacle)
{
    std::vector<Point3> points;
    add_surface(points, 0.0, 3.0, 0.0, 3.0, [](double x, double)
                { return std::tan(40 * DEG) * x; });
    const auto observation = classify_terrain(points, test_grid(), ClassifierParams{});
    EXPECT_EQ(cost_at(observation, 1.5, 1.5), cost::OBSTACLE);
}

TEST(TerrainClassifier, RockIsObstacleAndDistantGroundStaysFree)
{
    std::vector<Point3> points;
    auto rock = [](double x, double y)
    { return std::abs(x - 1.5) < 0.1 && std::abs(y - 1.5) < 0.1; };
    add_surface(points, 0.0, 3.0, 0.0, 3.0, [&](double x, double y)
                { return rock(x, y) ? 0.15 : 0.0; });
    const auto observation = classify_terrain(points, test_grid(), ClassifierParams{});
    EXPECT_EQ(cost_at(observation, 1.42, 1.5), cost::OBSTACLE);
    EXPECT_EQ(cost_at(observation, 1.5, 1.58), cost::OBSTACLE);
    // Step-down marks floor next to the rock too, out to about ground_window_m, but no further.
    EXPECT_EQ(cost_at(observation, 1.5, 2.0), cost::FREE);
    EXPECT_EQ(cost_at(observation, 0.5, 0.5), cost::FREE);
}

TEST(TerrainClassifier, HoleIsObstacle)
{
    std::vector<Point3> points;
    auto hole = [](double x, double y)
    { return std::abs(x - 1.5) < 0.15 && std::abs(y - 1.5) < 0.15; };
    add_surface(points, 0.0, 3.0, 0.0, 3.0, [&](double x, double y)
                { return hole(x, y) ? -0.15 : 0.0; });
    const auto observation = classify_terrain(points, test_grid(), ClassifierParams{});
    EXPECT_EQ(cost_at(observation, 1.5, 1.5), cost::OBSTACLE);
    EXPECT_EQ(cost_at(observation, 0.5, 2.5), cost::FREE);
}

TEST(TerrainClassifier, StepDownTestCanBeDisabled)
{
    std::vector<Point3> points;
    auto hole = [](double x, double y)
    { return std::abs(x - 1.5) < 0.15 && std::abs(y - 1.5) < 0.15; };
    add_surface(points, 0.0, 3.0, 0.0, 3.0, [&](double x, double y)
                { return hole(x, y) ? -0.15 : 0.0; });
    ClassifierParams params;
    params.max_step_down_m = 0.0;
    const auto observation = classify_terrain(points, test_grid(), params);
    // The hole floor is no longer an obstacle; its rim still is, as a step up from the floor.
    EXPECT_EQ(cost_at(observation, 1.5, 1.5), cost::FREE);
    EXPECT_EQ(cost_at(observation, 1.66, 1.5), cost::OBSTACLE);
}

TEST(TerrainClassifier, SparseCellsAreUnknown)
{
    std::vector<Point3> points = {{1.01F, 1.01F, 0.0F}, {1.02F, 1.02F, 0.0F}};
    const auto observation = classify_terrain(points, test_grid(), ClassifierParams{});
    EXPECT_EQ(cost_at(observation, 1.0, 1.0), cost::UNKNOWN);
    EXPECT_EQ(count_cost(observation, cost::UNKNOWN), 3600);
}

TEST(TerrainClassifier, OverheadReturnsAreIgnored)
{
    std::vector<Point3> points;
    add_surface(points, 0.0, 3.0, 0.0, 3.0, [](double, double)
                { return 0.0; });
    // A branch 0.8 m up, well above overhead_clearance_m.
    add_surface(points, 1.0, 2.0, 1.4, 1.6, [](double, double)
                { return 0.8; }, 0.02);
    const auto observation = classify_terrain(points, test_grid(), ClassifierParams{});
    EXPECT_EQ(count_cost(observation, cost::OBSTACLE), 0);
}

TEST(TerrainClassifier, ThinPostIsCaughtByHeightAboveGround)
{
    std::vector<Point3> points;
    add_surface(points, 0.0, 3.0, 0.0, 3.0, [](double, double)
                { return 0.0; });
    // A post whose few returns share a cell with plenty of floor: the floor percentile
    // still reads the floor, so only the height-above-ground test can see it.
    for (int i = 0; i < 5; ++i)
    {
        points.push_back({1.52F, 1.52F, 0.05F * static_cast<float>(i + 1)});
    }
    ClassifierParams params;
    params.min_obstacle_cells = 1;
    const auto observation = classify_terrain(points, test_grid(), params);
    EXPECT_EQ(cost_at(observation, 1.52, 1.52), cost::OBSTACLE);
}

TEST(TerrainClassifier, NoisyFlatGroundIsFree)
{
    // Stereo-like noise. The floor and top percentiles keep it from reading as rocks.
    std::mt19937 generator(42);
    std::normal_distribution<double> noise(0.0, 0.01);
    std::vector<Point3> points;
    add_surface(points, 0.0, 3.0, 0.0, 3.0, [&](double, double)
                { return noise(generator); });
    const auto observation = classify_terrain(points, test_grid(), ClassifierParams{});
    EXPECT_EQ(count_cost(observation, cost::OBSTACLE), 0);
}

TEST(TerrainClassifier, SmallBumpGetsARoughnessCost)
{
    std::vector<Point3> points;
    // Edges at x = 1.425 / 1.575, mid-cell. A cell straddling an edge holds both floor
    // and top returns, so height-above-ground sees the bump's full height. (A cell wholly
    // on top would not: the slope allowance, tan(25 deg) * 5 cm = 2.3 cm per cell, hides
    // most of a 4 cm step between neighbouring cells.)
    auto bump = [](double x, double y)
    { return std::abs(x - 1.5) < 0.075 && std::abs(y - 1.5) < 0.075; };
    // 4 cm: two thirds of max_step_up_m, so above rough_start_fraction but not lethal.
    add_surface(points, 0.0, 3.0, 0.0, 3.0, [&](double x, double y)
                { return bump(x, y) ? 0.04 : 0.0; });
    const auto observation = classify_terrain(points, test_grid(), ClassifierParams{});
    const auto value = cost_at(observation, 1.42, 1.5);
    EXPECT_GT(value, cost::FREE);
    EXPECT_LT(value, cost::OBSTACLE);
    EXPECT_EQ(cost_at(observation, 0.5, 0.5), cost::FREE);
}

TEST(TerrainClassifier, PointsOutsideTheGridAreIgnored)
{
    std::vector<Point3> points;
    add_surface(points, -2.0, -1.0, -2.0, -1.0, [](double, double)
                { return 0.0; });
    points.push_back({std::nanf(""), 1.0F, 0.0F});
    const auto observation = classify_terrain(points, test_grid(), ClassifierParams{});
    EXPECT_EQ(count_cost(observation, cost::UNKNOWN), 3600);
}

TEST(RemoveSmallObstacles, ErasesSpecklesKeepsDiagonalPairs)
{
    GridGeometry geometry;
    geometry.width = 5;
    geometry.height = 5;
    std::vector<std::int8_t> grid(25, cost::FREE);
    grid[geometry.index(0, 0)] = cost::OBSTACLE;  // lone speckle
    grid[geometry.index(2, 2)] = cost::OBSTACLE;  // diagonal pair
    grid[geometry.index(3, 3)] = cost::OBSTACLE;
    remove_small_obstacles(grid, geometry, 2);
    EXPECT_EQ(grid[geometry.index(0, 0)], cost::FREE);
    EXPECT_EQ(grid[geometry.index(2, 2)], cost::OBSTACLE);
    EXPECT_EQ(grid[geometry.index(3, 3)], cost::OBSTACLE);
}
