#include "bias_remover/stationary_gate.hpp"

#include <gtest/gtest.h>

#include <optional>

using imu_processors::combine_stationary_votes;
using imu_processors::gyro_within_bias_limit;
using imu_processors::parse_stationary_mode;
using imu_processors::StationaryMode;

namespace
{
    constexpr std::optional<bool> DISABLED = std::nullopt;
}

TEST(StationaryGateTest, NoEnabledSourceIsNeverStationary)
{
    // Regression: disabled sources used to count as "ok", so the default
    // config (both disabled) learned bias continuously, even while turning.
    EXPECT_FALSE(combine_stationary_votes(StationaryMode::OR, DISABLED, DISABLED));
    EXPECT_FALSE(combine_stationary_votes(StationaryMode::AND, DISABLED, DISABLED));
}

TEST(StationaryGateTest, DisabledSourceDoesNotSatisfyOrMode)
{
    // Regression: OR with one disabled source was always true.
    EXPECT_FALSE(combine_stationary_votes(StationaryMode::OR, DISABLED, false));
    EXPECT_FALSE(combine_stationary_votes(StationaryMode::OR, false, DISABLED));
    EXPECT_TRUE(combine_stationary_votes(StationaryMode::OR, DISABLED, true));
}

TEST(StationaryGateTest, SingleEnabledSourceDecidesInAndMode)
{
    EXPECT_TRUE(combine_stationary_votes(StationaryMode::AND, DISABLED, true));
    EXPECT_FALSE(combine_stationary_votes(StationaryMode::AND, DISABLED, false));
    EXPECT_TRUE(combine_stationary_votes(StationaryMode::AND, true, DISABLED));
}

TEST(StationaryGateTest, TwoSourcesCombine)
{
    EXPECT_TRUE(combine_stationary_votes(StationaryMode::AND, true, true));
    EXPECT_FALSE(combine_stationary_votes(StationaryMode::AND, true, false));
    EXPECT_TRUE(combine_stationary_votes(StationaryMode::OR, true, false));
    EXPECT_FALSE(combine_stationary_votes(StationaryMode::OR, false, false));
}

TEST(StationaryGateTest, ParsesModeCaseInsensitively)
{
    EXPECT_EQ(parse_stationary_mode("AND"), StationaryMode::AND);
    EXPECT_EQ(parse_stationary_mode("or"), StationaryMode::OR);
    EXPECT_FALSE(parse_stationary_mode("XOR").has_value());
    EXPECT_FALSE(parse_stationary_mode("").has_value());
}

TEST(StationaryGateTest, GyroLimitVetoesLargeRates)
{
    EXPECT_TRUE(gyro_within_bias_limit(0.01, -0.02, 0.005, 0.1));
    EXPECT_FALSE(gyro_within_bias_limit(0.0, 0.0, 0.3, 0.1));
    EXPECT_FALSE(gyro_within_bias_limit(-0.2, 0.0, 0.0, 0.1));
    // <= 0 disables the check
    EXPECT_TRUE(gyro_within_bias_limit(5.0, 5.0, 5.0, 0.0));
}
