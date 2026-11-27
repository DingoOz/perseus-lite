#include "perseus_terrain/cost_mapping.hpp"

#include <gtest/gtest.h>

#include <vector>

using perseus_terrain::combine_max;
using perseus_terrain::GridGeometry;
using perseus_terrain::occupancy_to_cost;
using perseus_terrain::sample_worst;
namespace value = perseus_terrain::costmap_value;

TEST(CostMapping, OccupancyToCost)
{
    EXPECT_EQ(occupancy_to_cost(-1, 100), value::NO_INFORMATION);
    EXPECT_EQ(occupancy_to_cost(0, 100), value::FREE_SPACE);
    EXPECT_EQ(occupancy_to_cost(100, 100), value::LETHAL_OBSTACLE);
    EXPECT_EQ(occupancy_to_cost(99, 100), value::MAX_NON_OBSTACLE);
    EXPECT_EQ(occupancy_to_cost(50, 100), 127);
    // A lower lethal threshold makes rough cells lethal.
    EXPECT_EQ(occupancy_to_cost(60, 50), value::LETHAL_OBSTACLE);
}

TEST(CostMapping, SampleWorstCoversTheWholeSquare)
{
    GridGeometry geometry;
    geometry.resolution = 0.05;
    geometry.width = 4;
    geometry.height = 4;
    std::vector<std::int8_t> data(16, -1);
    data[geometry.index(1, 1)] = 0;
    data[geometry.index(2, 1)] = 100;
    // A 0.1 m square centred on the corner shared by cells (1,1), (2,1), (1,2), (2,2).
    EXPECT_EQ(sample_worst(geometry, data, 0.1, 0.1, 0.1), 100);
    // A one-cell square exactly on cell (1, 1) sees only that cell.
    EXPECT_EQ(sample_worst(geometry, data, 0.075, 0.075, 0.05), 0);
    // Only unknown cells under it.
    EXPECT_EQ(sample_worst(geometry, data, 0.175, 0.175, 0.05), -1);
    // Entirely outside.
    EXPECT_EQ(sample_worst(geometry, data, 5.0, 5.0, 0.05), -1);
}

TEST(CostMapping, CombineMax)
{
    EXPECT_EQ(combine_max(value::NO_INFORMATION, 10), 10);
    EXPECT_EQ(combine_max(20, value::NO_INFORMATION), 20);
    EXPECT_EQ(combine_max(20, 10), 20);
    EXPECT_EQ(combine_max(20, value::LETHAL_OBSTACLE), value::LETHAL_OBSTACLE);
}
