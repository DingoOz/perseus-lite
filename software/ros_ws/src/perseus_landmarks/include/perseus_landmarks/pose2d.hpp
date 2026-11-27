#pragma once

/// @file pose2d.hpp
/// @brief Planar rigid transforms (x, y, yaw).

#include <cmath>

namespace perseus_landmarks
{
    /// @brief Wraps an angle to (-pi, pi].
    inline double wrap_angle(double angle)
    {
        angle = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (angle <= 0.0)
        {
            angle += 2.0 * M_PI;
        }
        return angle - M_PI;
    }

    /// @brief A planar point.
    struct Point2D
    {
        double x{0.0};
        double y{0.0};
    };

    /// @brief A planar rigid transform. As "a_from_b" it maps points expressed in frame b
    ///        into frame a: p_a = R(yaw) p_b + (x, y).
    struct Pose2D
    {
        double x{0.0};
        double y{0.0};
        double yaw{0.0};

        /// @brief Applies the transform to a point.
        Point2D apply(const Point2D& point) const
        {
            const double c = std::cos(yaw);
            const double s = std::sin(yaw);
            return {c * point.x - s * point.y + x, s * point.x + c * point.y + y};
        }

        /// @brief The inverse transform.
        Pose2D inverse() const
        {
            const double c = std::cos(yaw);
            const double s = std::sin(yaw);
            return {-(c * x + s * y), -(-s * x + c * y), wrap_angle(-yaw)};
        }

        /// @brief this * other: first other, then this.
        Pose2D compose(const Pose2D& other) const
        {
            const Point2D t = apply({other.x, other.y});
            return {t.x, t.y, wrap_angle(yaw + other.yaw)};
        }
    };

}
