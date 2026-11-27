#include "perseus_terrain/depth_projection.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>

namespace perseus_terrain
{
    std::optional<DepthEncoding> depth_encoding_from_string(const std::string& encoding)
    {
        if (encoding == "16UC1" || encoding == "mono16")
        {
            return DepthEncoding::UINT16_MILLIMETRES;
        }
        if (encoding == "32FC1")
        {
            return DepthEncoding::FLOAT32_METRES;
        }
        return std::nullopt;
    }

    namespace
    {
        /// @brief Depth in metres at one pixel; NaN when the pixel has no data.
        float depth_at(const DepthImageView& image, int u, int v)
        {
            const std::uint8_t* row = image.data + static_cast<std::size_t>(v) * image.step;
            const bool swap = image.big_endian != (std::endian::native == std::endian::big);
            if (image.encoding == DepthEncoding::UINT16_MILLIMETRES)
            {
                std::uint16_t raw = 0;
                std::memcpy(&raw, row + static_cast<std::size_t>(u) * sizeof(raw), sizeof(raw));
                if (swap)
                {
                    raw = static_cast<std::uint16_t>((raw >> 8) | (raw << 8));
                }
                return raw == 0 ? std::nanf("") : static_cast<float>(raw) * 0.001F;
            }
            std::uint32_t bits = 0;
            std::memcpy(&bits, row + static_cast<std::size_t>(u) * sizeof(bits), sizeof(bits));
            if (swap)
            {
                bits = ((bits & 0x000000FFU) << 24) | ((bits & 0x0000FF00U) << 8) | ((bits & 0x00FF0000U) >> 8) |
                       ((bits & 0xFF000000U) >> 24);
            }
            float metres = 0.0F;
            std::memcpy(&metres, &bits, sizeof(metres));
            return (std::isfinite(metres) && metres > 0.0F) ? metres : std::nanf("");
        }
    }

    std::vector<Point3> project_depth(const DepthImageView& image, const PinholeIntrinsics& intrinsics,
                                      const DepthProjectionParams& params)
    {
        std::vector<Point3> points;
        if (image.data == nullptr || image.width <= 0 || image.height <= 0 || intrinsics.fx <= 0.0 ||
            intrinsics.fy <= 0.0)
        {
            return points;
        }
        const std::size_t bytes_per_pixel = image.encoding == DepthEncoding::UINT16_MILLIMETRES ? 2 : 4;
        if (image.step < static_cast<std::size_t>(image.width) * bytes_per_pixel)
        {
            return points;
        }
        const int stride = std::max(1, params.pixel_stride);
        const float min_depth = static_cast<float>(params.min_depth_m);
        const float max_depth = static_cast<float>(params.max_depth_m);
        const double inverse_fx = 1.0 / intrinsics.fx;
        const double inverse_fy = 1.0 / intrinsics.fy;
        points.reserve(static_cast<std::size_t>((image.width / stride + 1) * (image.height / stride + 1)));
        // Start half a stride in, so the sampled pixels are centred in the image.
        for (int v = stride / 2; v < image.height; v += stride)
        {
            const double ray_y = (static_cast<double>(v) - intrinsics.cy) * inverse_fy;
            for (int u = stride / 2; u < image.width; u += stride)
            {
                const float depth = depth_at(image, u, v);
                if (std::isnan(depth) || depth < min_depth || depth > max_depth)
                {
                    continue;
                }
                const double ray_x = (static_cast<double>(u) - intrinsics.cx) * inverse_fx;
                points.push_back({static_cast<float>(ray_x * depth), static_cast<float>(ray_y * depth), depth});
            }
        }
        return points;
    }

}
