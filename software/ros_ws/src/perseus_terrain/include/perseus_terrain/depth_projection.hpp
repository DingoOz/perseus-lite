#pragma once

/// @file depth_projection.hpp
/// @brief Back-projection of a (decimated) depth image into 3D points.
///
/// The terrain mapper reads the camera's depth image directly instead of a
/// PointCloud2: projecting every Nth pixel on the robot is far cheaper than having the
/// driver build a full-resolution cloud and ship it between processes, and the
/// classifier's cells are much coarser than a pixel anyway.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "perseus_terrain/terrain_classifier.hpp"

namespace perseus_terrain
{
    /// @brief Pixel formats a depth image may arrive in.
    enum class DepthEncoding
    {
        /// @brief 16UC1 / mono16: unsigned millimetres, 0 = no data. The OAK-D driver.
        UINT16_MILLIMETRES,
        /// @brief 32FC1: float metres, NaN/inf/0 = no data. Gazebo.
        FLOAT32_METRES,
    };

    /// @brief Parses a sensor_msgs/Image encoding string.
    std::optional<DepthEncoding> depth_encoding_from_string(const std::string& encoding);

    /// @brief Pinhole intrinsics of the (rectified) depth image.
    struct PinholeIntrinsics
    {
        double fx{0.0};
        double fy{0.0};
        double cx{0.0};
        double cy{0.0};
    };

    /// @brief Which pixels to project and which depths to trust.
    struct DepthProjectionParams
    {
        /// @brief Project every Nth pixel along both axes.
        int pixel_stride{4};
        /// @brief Depths closer than this are dropped (below the stereo minimum).
        double min_depth_m{0.3};
        /// @brief Depths further than this are dropped. Stereo depth error grows with the
        ///        square of the distance, so far returns are too noisy to classify.
        double max_depth_m{3.5};
    };

    /// @brief A view of one depth image's pixel buffer.
    struct DepthImageView
    {
        const std::uint8_t* data{nullptr};
        std::size_t step{0};
        int width{0};
        int height{0};
        DepthEncoding encoding{DepthEncoding::UINT16_MILLIMETRES};
        bool big_endian{false};
    };

    /// @brief Projects the image into points in its optical frame (x right, y down,
    ///        z forward, the REP-103 camera optical convention).
    std::vector<Point3> project_depth(const DepthImageView& image, const PinholeIntrinsics& intrinsics,
                                      const DepthProjectionParams& params);

}
