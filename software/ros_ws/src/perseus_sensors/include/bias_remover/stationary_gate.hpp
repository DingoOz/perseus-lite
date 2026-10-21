#pragma once

#include <cmath>
#include <optional>
#include <string>

namespace imu_processors
{
    /// How the enabled motion sources are combined into a "stationary" decision
    enum class StationaryMode
    {
        AND,  ///< every enabled source must report zero motion
        OR    ///< at least one enabled source must report zero motion
    };

    /// Parse "AND"/"OR" (case-insensitive). Returns std::nullopt for anything else.
    inline std::optional<StationaryMode> parse_stationary_mode(const std::string& mode)
    {
        std::string upper;
        upper.reserve(mode.size());
        for (const char c : mode)
        {
            upper.push_back(static_cast<char>((c >= 'a' && c <= 'z') ? c - 'a' + 'A' : c));
        }
        if (upper == "AND")
            return StationaryMode::AND;
        if (upper == "OR")
            return StationaryMode::OR;
        return std::nullopt;
    }

    /**
     * @brief Combine per-source stationary votes
     *
     * Only enabled sources vote: a disabled source is std::nullopt and is ignored
     * (it must never count as "stationary", otherwise one disabled source makes
     * OR mode always true). An enabled source with no fresh data must pass
     * `false`. With no enabled source at all the robot is never considered
     * stationary, so no bias is learned.
     *
     * @param mode AND / OR policy across the enabled sources
     * @param cmd_vel_vote cmd_vel vote, or std::nullopt if the source is disabled
     * @param odom_vote odometry vote, or std::nullopt if the source is disabled
     */
    inline bool combine_stationary_votes(StationaryMode mode,
                                         const std::optional<bool>& cmd_vel_vote,
                                         const std::optional<bool>& odom_vote)
    {
        bool any_enabled = false;
        bool all_stationary = true;
        bool some_stationary = false;
        for (const auto& vote : {cmd_vel_vote, odom_vote})
        {
            if (!vote)
                continue;
            any_enabled = true;
            all_stationary = all_stationary && *vote;
            some_stationary = some_stationary || *vote;
        }
        if (!any_enabled)
            return false;
        return (mode == StationaryMode::AND) ? all_stationary : some_stationary;
    }

    /**
     * @brief Veto samples whose rotation rate is too large to be gyro bias
     *
     * @param max_rate Largest per-axis |rate| (rad/s) accepted as bias; <= 0 disables the check
     */
    inline bool gyro_within_bias_limit(double x, double y, double z, double max_rate)
    {
        if (max_rate <= 0.0)
            return true;
        return std::abs(x) <= max_rate && std::abs(y) <= max_rate && std::abs(z) <= max_rate;
    }

}  // namespace imu_processors
