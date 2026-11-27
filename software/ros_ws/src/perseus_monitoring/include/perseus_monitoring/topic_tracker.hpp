#pragma once

/// @file topic_tracker.hpp
/// @brief Rate, bandwidth and age of one topic over a sliding window, and the verdict
///        drawn from them. Pure logic, unit-tested without ROS.

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

namespace perseus_monitoring
{
    /// @brief Verdicts, numerically equal to perseus_interfaces/TopicHealth STATUS_*.
    enum class TopicStatus : std::uint8_t
    {
        OK = 0,
        SLOW = 1,
        STALE = 2,
        NO_PUBLISHER = 3,
    };

    /// @brief Sliding-window statistics of one topic.
    class TopicTracker
    {
    public:
        explicit TopicTracker(double window_s);

        /// @brief Records one message of @p bytes at time @p now_s.
        void record(double now_s, std::size_t bytes);

        /// @brief Messages per second over the window ending at @p now_s.
        double rate_hz(double now_s);
        /// @brief Bytes per second over the window ending at @p now_s.
        double bandwidth(double now_s);
        /// @brief Seconds since the last message, or nullopt if none has arrived.
        std::optional<double> age_s(double now_s) const;

    private:
        void prune(double now_s);

        struct Arrival
        {
            double time_s;
            std::size_t bytes;
        };

        double window_s_;
        std::deque<Arrival> arrivals_;
        std::optional<double> last_s_;
    };

    /// @brief The verdict for one topic.
    /// @param publisher_count Publishers currently in the graph.
    /// @param age_s Seconds since the last message, nullopt if never.
    /// @param rate_hz Measured rate.
    /// @param expected_rate_hz Expected rate; 0 disables the rate check.
    /// @param rate_tolerance SLOW below rate_tolerance * expected_rate_hz.
    /// @param stale_timeout_s STALE when silent for longer than this.
    TopicStatus classify_topic(std::size_t publisher_count, std::optional<double> age_s, double rate_hz,
                               double expected_rate_hz, double rate_tolerance, double stale_timeout_s);

}
