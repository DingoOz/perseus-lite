#include "perseus_terrain/depth_projection.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <vector>

using perseus_terrain::depth_encoding_from_string;
using perseus_terrain::DepthEncoding;
using perseus_terrain::DepthImageView;
using perseus_terrain::DepthProjectionParams;
using perseus_terrain::PinholeIntrinsics;
using perseus_terrain::project_depth;

namespace
{
    // A 4 x 3 image with the principal point between the middle columns.
    const PinholeIntrinsics INTRINSICS{2.0, 2.0, 1.5, 1.0};

    DepthProjectionParams every_pixel()
    {
        DepthProjectionParams params;
        params.pixel_stride = 1;
        params.min_depth_m = 0.1;
        params.max_depth_m = 10.0;
        return params;
    }
}

TEST(DepthProjection, ParsesEncodings)
{
    EXPECT_EQ(depth_encoding_from_string("16UC1"), DepthEncoding::UINT16_MILLIMETRES);
    EXPECT_EQ(depth_encoding_from_string("mono16"), DepthEncoding::UINT16_MILLIMETRES);
    EXPECT_EQ(depth_encoding_from_string("32FC1"), DepthEncoding::FLOAT32_METRES);
    EXPECT_FALSE(depth_encoding_from_string("rgb8").has_value());
}

TEST(DepthProjection, ProjectsMillimetreImage)
{
    std::vector<std::uint16_t> pixels(12, 0);  // 0 = no data
    pixels[1 * 4 + 3] = 2000;                  // (u 3, v 1) at 2 m
    DepthImageView view{reinterpret_cast<const std::uint8_t*>(pixels.data()), 4 * sizeof(std::uint16_t), 4, 3,
                        DepthEncoding::UINT16_MILLIMETRES, false};
    const auto points = project_depth(view, INTRINSICS, every_pixel());
    ASSERT_EQ(points.size(), 1U);
    EXPECT_FLOAT_EQ(points[0].x, 1.5F);  // (3 - 1.5) / 2 * 2
    EXPECT_FLOAT_EQ(points[0].y, 0.0F);
    EXPECT_FLOAT_EQ(points[0].z, 2.0F);
}

TEST(DepthProjection, ProjectsFloatImageAndSkipsInvalid)
{
    std::vector<float> pixels(12, std::nanf(""));
    pixels[0] = 1.0F;           // (0, 0)
    pixels[1] = 0.0F;           // zero: no data
    pixels[2] = 1.0F / 0.0F;    // inf: no data
    pixels[2 * 4 + 1] = 0.05F;  // below min depth
    pixels[2 * 4 + 2] = 20.0F;  // beyond max depth
    DepthImageView view{reinterpret_cast<const std::uint8_t*>(pixels.data()), 4 * sizeof(float), 4, 3,
                        DepthEncoding::FLOAT32_METRES, false};
    const auto points = project_depth(view, INTRINSICS, every_pixel());
    ASSERT_EQ(points.size(), 1U);
    EXPECT_FLOAT_EQ(points[0].x, -0.75F);
    EXPECT_FLOAT_EQ(points[0].y, -0.5F);
    EXPECT_FLOAT_EQ(points[0].z, 1.0F);
}

TEST(DepthProjection, HonoursStride)
{
    std::vector<std::uint16_t> pixels(12, 1000);
    DepthImageView view{reinterpret_cast<const std::uint8_t*>(pixels.data()), 4 * sizeof(std::uint16_t), 4, 3,
                        DepthEncoding::UINT16_MILLIMETRES, false};
    DepthProjectionParams params = every_pixel();
    params.pixel_stride = 2;
    // Starts half a stride in: columns 1 and 3, rows 1 only (3 rows).
    EXPECT_EQ(project_depth(view, INTRINSICS, params).size(), 2U);
}

TEST(DepthProjection, SwapsBigEndian)
{
    std::vector<std::uint8_t> bytes(4 * 3 * 2, 0);
    // 2000 = 0x07D0, written big-endian at (u 3, v 1).
    bytes[(1 * 4 + 3) * 2] = 0x07;
    bytes[(1 * 4 + 3) * 2 + 1] = 0xD0;
    DepthImageView view{bytes.data(), 4 * 2, 4, 3, DepthEncoding::UINT16_MILLIMETRES, true};
    const auto points = project_depth(view, INTRINSICS, every_pixel());
    ASSERT_EQ(points.size(), 1U);
    EXPECT_FLOAT_EQ(points[0].z, 2.0F);
}

TEST(DepthProjection, RejectsBadInputs)
{
    std::vector<std::uint16_t> pixels(12, 1000);
    DepthImageView view{reinterpret_cast<const std::uint8_t*>(pixels.data()), 2, 4, 3,
                        DepthEncoding::UINT16_MILLIMETRES, false};  // step too short
    EXPECT_TRUE(project_depth(view, INTRINSICS, every_pixel()).empty());
    view.step = 8;
    EXPECT_TRUE(project_depth(view, PinholeIntrinsics{}, every_pixel()).empty());
}
