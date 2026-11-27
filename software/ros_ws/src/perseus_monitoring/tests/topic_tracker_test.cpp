#include "perseus_monitoring/topic_tracker.hpp"

#include <gtest/gtest.h>

using perseus_monitoring::classify_topic;
using perseus_monitoring::TopicStatus;
using perseus_monitoring::TopicTracker;

TEST(TopicTracker, RateBandwidthAge)
{
    TopicTracker tracker(2.0);
    EXPECT_FALSE(tracker.age_s(0.0).has_value());
    for (int i = 0; i < 40; ++i)
    {
        tracker.record(0.1 * i, 100);  // 10 Hz, 100 B
    }
    EXPECT_NEAR(tracker.rate_hz(3.95), 10.0, 0.5);
    EXPECT_NEAR(tracker.bandwidth(3.95), 1000.0, 50.0);
    EXPECT_NEAR(*tracker.age_s(4.4), 0.5, 1e-9);
    // Everything ages out of the window.
    EXPECT_EQ(tracker.rate_hz(10.0), 0.0);
}

TEST(TopicTracker, Classification)
{
    EXPECT_EQ(classify_topic(0, 0.1, 10.0, 10.0, 0.7, 2.0), TopicStatus::NO_PUBLISHER);
    EXPECT_EQ(classify_topic(1, std::nullopt, 0.0, 10.0, 0.7, 2.0), TopicStatus::STALE);
    EXPECT_EQ(classify_topic(1, 3.0, 0.0, 10.0, 0.7, 2.0), TopicStatus::STALE);
    EXPECT_EQ(classify_topic(1, 0.1, 5.0, 10.0, 0.7, 2.0), TopicStatus::SLOW);
    EXPECT_EQ(classify_topic(1, 0.1, 8.0, 10.0, 0.7, 2.0), TopicStatus::OK);
    // Rate check disabled.
    EXPECT_EQ(classify_topic(1, 0.1, 0.5, 0.0, 0.7, 2.0), TopicStatus::OK);
}
