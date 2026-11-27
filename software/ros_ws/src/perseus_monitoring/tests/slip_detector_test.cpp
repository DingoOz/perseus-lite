#include "perseus_monitoring/slip_detector.hpp"

#include <gtest/gtest.h>

using perseus_monitoring::SlipDetector;
using perseus_monitoring::SlipDetectorConfig;

namespace
{
    /// @brief Feeds a constant command/measurement at 10 Hz from @p start for @p duration.
    double run(SlipDetector& detector, double commanded, double measured, double start, double duration)
    {
        double t = start;
        for (; t < start + duration; t += 0.1)
        {
            detector.update(commanded, measured, t);
        }
        return t;
    }
}

TEST(SlipDetector, TracksWhenAchieved)
{
    SlipDetector detector(SlipDetectorConfig{});
    run(detector, 0.2, 0.19, 0.0, 2.0);
    EXPECT_TRUE(detector.evaluating());
    EXPECT_FALSE(detector.fault());
    EXPECT_NEAR(detector.ratio(), 0.05, 1e-6);
}

TEST(SlipDetector, FaultsOnSustainedShortfall)
{
    SlipDetector detector(SlipDetectorConfig{});
    run(detector, 0.2, 0.05, 0.0, 2.0);
    EXPECT_TRUE(detector.fault());
    EXPECT_NEAR(detector.ratio(), 0.75, 1e-6);
}

TEST(SlipDetector, GracePeriodHidesAcceleration)
{
    SlipDetectorConfig config;
    config.grace_s = 0.5;
    SlipDetector detector(config);
    run(detector, 0.2, 0.0, 0.0, 0.45);
    EXPECT_FALSE(detector.evaluating());
    EXPECT_FALSE(detector.fault());
}

TEST(SlipDetector, SmallCommandsAreIgnoredAndReset)
{
    SlipDetector detector(SlipDetectorConfig{});
    run(detector, 0.2, 0.0, 0.0, 2.0);
    ASSERT_TRUE(detector.fault());
    detector.update(0.01, 0.0, 2.1);
    EXPECT_FALSE(detector.evaluating());
    EXPECT_FALSE(detector.fault());
}

TEST(SlipDetector, ReversalRestartsGrace)
{
    SlipDetector detector(SlipDetectorConfig{});
    const double t = run(detector, 0.2, 0.2, 0.0, 2.0);
    // Reversing: measured still positive while decelerating. Must not fault.
    run(detector, -0.2, 0.1, t, 0.4);
    EXPECT_FALSE(detector.evaluating());
    EXPECT_FALSE(detector.fault());
}

TEST(SlipDetector, WorksForNegativeCommands)
{
    SlipDetector detector(SlipDetectorConfig{});
    run(detector, -0.3, -0.05, 0.0, 2.0);
    EXPECT_TRUE(detector.fault());
}
