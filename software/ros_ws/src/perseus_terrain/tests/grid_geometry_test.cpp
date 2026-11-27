#include "perseus_terrain/grid_geometry.hpp"

#include <gtest/gtest.h>

#include <cmath>

#include "perseus_terrain/footprint.hpp"

using perseus_terrain::centred_window;
using perseus_terrain::GridGeometry;
using perseus_terrain::pad_polygon;
using perseus_terrain::polygon_contains;
using perseus_terrain::polygon_from_flat;
using perseus_terrain::transform_polygon;

TEST(GridGeometry, WorldToCellAndBack)
{
    GridGeometry geometry;
    geometry.origin_x = -1.0;
    geometry.origin_y = -1.0;
    geometry.resolution = 0.1;
    geometry.width = 20;
    geometry.height = 20;
    int cx = 0;
    int cy = 0;
    EXPECT_TRUE(geometry.world_to_cell(-0.95, 0.05, cx, cy));
    EXPECT_EQ(cx, 0);
    EXPECT_EQ(cy, 10);
    double x = 0.0;
    double y = 0.0;
    geometry.cell_center(cx, cy, x, y);
    EXPECT_NEAR(x, -0.95, 1e-9);
    EXPECT_NEAR(y, 0.05, 1e-9);
    EXPECT_FALSE(geometry.world_to_cell(-1.01, 0.0, cx, cy));
    EXPECT_FALSE(geometry.world_to_cell(1.0, 0.0, cx, cy));
    EXPECT_EQ(geometry.size(), 400U);
}

TEST(GridGeometry, CentredWindowsShareOneLattice)
{
    const GridGeometry a = centred_window(0.012, 3.98, 2.0, 0.05);
    const GridGeometry b = centred_window(0.493, 3.71, 2.0, 0.05);
    EXPECT_NEAR(std::remainder(a.origin_x - b.origin_x, 0.05), 0.0, 1e-9);
    EXPECT_NEAR(std::remainder(a.origin_y - b.origin_y, 0.05), 0.0, 1e-9);
    EXPECT_EQ(a.width, 80);
    int cx = 0;
    int cy = 0;
    EXPECT_TRUE(a.world_to_cell(0.012, 3.98, cx, cy));
    EXPECT_NEAR(cx, 40, 1);
}

TEST(Footprint, BuildPadPlaceAndTest)
{
    const auto square = polygon_from_flat({0.5, 0.5, 0.5, -0.5, -0.5, -0.5, -0.5, 0.5});
    ASSERT_EQ(square.size(), 4U);
    EXPECT_TRUE(polygon_contains(square, 0.0, 0.0));
    EXPECT_FALSE(polygon_contains(square, 0.6, 0.0));
    EXPECT_TRUE(polygon_contains(pad_polygon(square, 0.2), 0.6, 0.0));
    const auto moved = transform_polygon(square, 10.0, 0.0, M_PI / 4);
    EXPECT_TRUE(polygon_contains(moved, 10.0, 0.65));  // a corner rotated onto +y
    EXPECT_FALSE(polygon_contains(moved, 10.0 + 0.55, 0.55));
    EXPECT_TRUE(polygon_from_flat({1.0, 2.0, 3.0}).empty());
}
