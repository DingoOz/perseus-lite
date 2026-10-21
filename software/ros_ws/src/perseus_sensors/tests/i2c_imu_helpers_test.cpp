#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "i2c_imu_driver/imu_data_types.hpp"
#include "i2c_imu_driver/imu_device_config.hpp"

using i2c_imu_driver::covariance_from_param;
using i2c_imu_driver::ImuDeviceRegistry;

TEST(CovarianceTest, NineElementMatrixIsCopiedVerbatim)
{
    const std::vector<double> in{1, 2, 3, 4, 5, 6, 7, 8, 9};
    const auto out = covariance_from_param(in);
    ASSERT_TRUE(out.has_value());
    for (size_t i = 0; i < 9; ++i)
    {
        EXPECT_DOUBLE_EQ((*out)[i], in[i]);
    }
}

TEST(CovarianceTest, ThreeElementDiagonalIsExpanded)
{
    const auto out = covariance_from_param({0.1, 0.2, 0.3});
    ASSERT_TRUE(out.has_value());
    const std::array<double, 9> expected{0.1, 0, 0, 0, 0.2, 0, 0, 0, 0.3};
    for (size_t i = 0; i < 9; ++i)
    {
        EXPECT_DOUBLE_EQ((*out)[i], expected[i]);
    }
}

TEST(CovarianceTest, WrongLengthIsRejected)
{
    EXPECT_FALSE(covariance_from_param({}).has_value());
    EXPECT_FALSE(covariance_from_param({0.1}).has_value());
    EXPECT_FALSE(covariance_from_param({0.1, 0.2, 0.3, 0.4}).has_value());
    EXPECT_FALSE(covariance_from_param(std::vector<double>(10, 0.1)).has_value());
}

TEST(CovarianceTest, NonFiniteOrNegativeDiagonalIsRejected)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(covariance_from_param({nan, 0.1, 0.1}).has_value());
    EXPECT_FALSE(covariance_from_param({0.1, inf, 0.1}).has_value());
    EXPECT_FALSE(covariance_from_param({0.1, 0.1, -0.1}).has_value());
    // Negative off-diagonal terms are legitimate correlations
    EXPECT_TRUE(covariance_from_param({1, -0.5, 0, -0.5, 1, 0, 0, 0, 1}).has_value());
}

TEST(DeviceRegistryTest, Mpu9250UsesItsOwnTemperatureConversion)
{
    const auto mpu9250 = ImuDeviceRegistry::get_device_config("mpu9250");
    const auto mpu6050 = ImuDeviceRegistry::get_device_config("mpu6050");
    ASSERT_TRUE(mpu9250.has_value());
    ASSERT_TRUE(mpu6050.has_value());
    EXPECT_NEAR(mpu9250->get().temp_scale_factor, 1.0 / 333.87, 1e-12);
    EXPECT_DOUBLE_EQ(mpu9250->get().temp_offset, 21.0);
    EXPECT_NEAR(mpu6050->get().temp_scale_factor, 1.0 / 340.0, 1e-12);
    EXPECT_DOUBLE_EQ(mpu6050->get().temp_offset, 36.53);
}

TEST(DeviceRegistryTest, DefaultDeviceIsRegistered)
{
    EXPECT_TRUE(ImuDeviceRegistry::get_device_config(ImuDeviceRegistry::get_default_device()).has_value());
    EXPECT_FALSE(ImuDeviceRegistry::get_device_config("not_a_device").has_value());
}
