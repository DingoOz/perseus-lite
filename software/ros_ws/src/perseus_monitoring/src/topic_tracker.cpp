#include "perseus_monitoring/topic_tracker.hpp"

#include <algorithm>

namespace perseus_monitoring
{
    TopicTracker::TopicTracker(double window_s)
        : window_s_(std::max(window_s, 1e-3))
    {
    }

    void TopicTracker::record(double now_s, std::size_t bytes)
    {
        arrivals_.push_back({now_s, bytes});
        last_s_ = now_s;
        prune(now_s);
    }

    void TopicTracker::prune(double now_s)
    {
        while (!arrivals_.empty() && now_s - arrivals_.front().time_s > window_s_)
        {
            arrivals_.pop_front();
        }
    }

    double TopicTracker::rate_hz(double now_s)
    {
        prune(now_s);
        return static_cast<double>(arrivals_.size()) / window_s_;
    }

    double TopicTracker::bandwidth(double now_s)
    {
        prune(now_s);
        std::size_t total = 0;
        for (const Arrival& arrival : arrivals_)
        {
            total += arrival.bytes;
        }
        return static_cast<double>(total) / window_s_;
    }

    std::optional<double> TopicTracker::age_s(double now_s) const
    {
        if (!last_s_)
        {
            return std::nullopt;
        }
        return now_s - *last_s_;
    }

    TopicStatus classify_topic(std::size_t publisher_count, std::optional<double> age_s, double rate_hz,
                               double expected_rate_hz, double rate_tolerance, double stale_timeout_s)
    {
        if (publisher_count == 0)
        {
            return TopicStatus::NO_PUBLISHER;
        }
        if (!age_s || *age_s > stale_timeout_s)
        {
            return TopicStatus::STALE;
        }
        if (expected_rate_hz > 0.0 && rate_hz < rate_tolerance * expected_rate_hz)
        {
            return TopicStatus::SLOW;
        }
        return TopicStatus::OK;
    }

}
