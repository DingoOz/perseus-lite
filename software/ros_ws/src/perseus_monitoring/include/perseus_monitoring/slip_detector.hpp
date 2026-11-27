#pragma once

/// @file slip_detector.hpp
/// @brief Sustained commanded-versus-measured mismatch on one motion channel.
///
/// Every command has a transient mismatch while the robot accelerates, and single
/// samples are noisy, so a fault is declared only when the mean shortfall over a sliding
/// window stays above a threshold, after a grace period from when the command started.
/// Pure logic: fed plain numbers and a time in seconds, unit-tested without ROS.

#include <deque>

namespace perseus_monitoring
{
    /// @brief Thresholds of one channel.
    struct SlipDetectorConfig
    {
        /// @brief Commands smaller than this (absolute) are ignored: too small to tell a
        ///        shortfall from noise. Also resets the channel.
        double min_command{0.05};
        /// @brief Mean shortfall (1 - measured/commanded) at or above which the channel
        ///        faults. 0.5 = achieving half the command or less.
        double fault_ratio{0.5};
        /// @brief Sliding window the shortfall is averaged over, in seconds.
        double window_s{1.0};
        /// @brief Time after a command starts (or reverses) before evaluation begins.
        double grace_s{0.5};
    };

    /// @brief See the file comment.
    class SlipDetector
    {
    public:
        explicit SlipDetector(SlipDetectorConfig config);

        /// @brief Feeds one sample.
        /// @param commanded What was asked for (signed, m/s or rad/s).
        /// @param measured What happened, same units and sign convention.
        /// @param now_s Sample time in seconds (any monotonic origin).
        void update(double commanded, double measured, double now_s);

        /// @brief Forgets everything, as if no command had ever been given. Call when
        ///        an input goes stale.
        void reset();

        /// @brief True once a command has been held past the grace period and the window
        ///        holds enough samples.
        bool evaluating() const;
        /// @brief True while evaluating and the mean shortfall >= fault_ratio.
        bool fault() const;
        /// @brief Mean shortfall over the window (0 while not evaluating).
        double ratio() const;
        double commanded() const;
        double measured() const;

    private:
        struct Sample
        {
            double time_s;
            double ratio;
        };

        SlipDetectorConfig config_;
        std::deque<Sample> samples_;
        bool command_active_{false};
        double active_since_s_{0.0};
        double command_sign_{0.0};
        double commanded_{0.0};
        double measured_{0.0};
        double ratio_{0.0};
        bool evaluating_{false};
    };

}
