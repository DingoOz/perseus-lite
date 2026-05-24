#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <string>
#include <unordered_set>
#include <vector>

class ControllerConfigTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        config_path = std::string(TEST_CONFIG_DIR) + "/perseus_lite_controllers.yaml";
    }

    std::string config_path;

    YAML::Node load_config()
    {
        return YAML::LoadFile(config_path);
    }
};

TEST_F(ControllerConfigTest, ConfigFileExists)
{
    std::ifstream file(config_path);
    ASSERT_TRUE(file.good()) << "Controller config not found at: " << config_path;
}

TEST_F(ControllerConfigTest, ControllerManagerExists)
{
    auto config = load_config();
    ASSERT_TRUE(config["controller_manager"])
        << "Missing 'controller_manager' top-level key";
    ASSERT_TRUE(config["controller_manager"]["ros__parameters"])
        << "Missing 'controller_manager.ros__parameters'";
}

TEST_F(ControllerConfigTest, UpdateRateIsPositive)
{
    auto config = load_config();
    auto params = config["controller_manager"]["ros__parameters"];
    ASSERT_TRUE(params["update_rate"]) << "Missing update_rate";
    EXPECT_GT(params["update_rate"].as<int>(), 0);
}

TEST_F(ControllerConfigTest, RequiredControllersExist)
{
    auto config = load_config();
    auto params = config["controller_manager"]["ros__parameters"];

    EXPECT_TRUE(params["joint_state_broadcaster"])
        << "Missing joint_state_broadcaster controller";
    EXPECT_TRUE(params["diff_drive_base_controller"])
        << "Missing diff_drive_base_controller controller";
}

TEST_F(ControllerConfigTest, ControllerTypes)
{
    auto config = load_config();
    auto params = config["controller_manager"]["ros__parameters"];

    auto jsb = params["joint_state_broadcaster"];
    ASSERT_TRUE(jsb["type"]) << "joint_state_broadcaster missing 'type'";
    EXPECT_EQ(jsb["type"].as<std::string>(),
              "joint_state_broadcaster/JointStateBroadcaster");

    auto ddc = params["diff_drive_base_controller"];
    ASSERT_TRUE(ddc["type"]) << "diff_drive_base_controller missing 'type'";
    EXPECT_EQ(ddc["type"].as<std::string>(),
              "diff_drive_controller/DiffDriveController");
}

TEST_F(ControllerConfigTest, DiffDriveParametersExist)
{
    auto config = load_config();
    ASSERT_TRUE(config["diff_drive_base_controller"])
        << "Missing diff_drive_base_controller config section";

    auto params = config["diff_drive_base_controller"]["ros__parameters"];
    ASSERT_TRUE(params) << "Missing diff_drive_base_controller.ros__parameters";

    EXPECT_TRUE(params["left_wheel_names"]) << "Missing left_wheel_names";
    EXPECT_TRUE(params["right_wheel_names"]) << "Missing right_wheel_names";
    EXPECT_TRUE(params["wheel_separation"]) << "Missing wheel_separation";
    EXPECT_TRUE(params["wheel_radius"]) << "Missing wheel_radius";
}

TEST_F(ControllerConfigTest, WheelNamesMatchExpected)
{
    auto config = load_config();
    auto params = config["diff_drive_base_controller"]["ros__parameters"];

    auto left_wheels = params["left_wheel_names"];
    auto right_wheels = params["right_wheel_names"];

    ASSERT_EQ(left_wheels.size(), 2u) << "Expected 2 left wheel joints";
    ASSERT_EQ(right_wheels.size(), 2u) << "Expected 2 right wheel joints";

    std::unordered_set<std::string> left_names;
    for (const auto& name : left_wheels)
    {
        left_names.insert(name.as<std::string>());
    }

    std::unordered_set<std::string> right_names;
    for (const auto& name : right_wheels)
    {
        right_names.insert(name.as<std::string>());
    }

    EXPECT_TRUE(left_names.count("front_left_wheel_joint"))
        << "Missing front_left_wheel_joint in left wheels";
    EXPECT_TRUE(left_names.count("rear_left_wheel_joint"))
        << "Missing rear_left_wheel_joint in left wheels";
    EXPECT_TRUE(right_names.count("front_right_wheel_joint"))
        << "Missing front_right_wheel_joint in right wheels";
    EXPECT_TRUE(right_names.count("rear_right_wheel_joint"))
        << "Missing rear_right_wheel_joint in right wheels";
}

TEST_F(ControllerConfigTest, WheelGeometryIsReasonable)
{
    auto config = load_config();
    auto params = config["diff_drive_base_controller"]["ros__parameters"];

    double separation = params["wheel_separation"].as<double>();
    double radius = params["wheel_radius"].as<double>();

    EXPECT_GT(separation, 0.1) << "Wheel separation too small";
    EXPECT_LT(separation, 2.0) << "Wheel separation too large for a lite rover";

    EXPECT_GT(radius, 0.01) << "Wheel radius too small";
    EXPECT_LT(radius, 0.5) << "Wheel radius too large for a lite rover";
}

TEST_F(ControllerConfigTest, WheelsPerSideMatchesConfig)
{
    auto config = load_config();
    auto params = config["diff_drive_base_controller"]["ros__parameters"];

    ASSERT_TRUE(params["wheels_per_side"]) << "Missing wheels_per_side";
    int wheels_per_side = params["wheels_per_side"].as<int>();

    auto left_count = params["left_wheel_names"].size();
    auto right_count = params["right_wheel_names"].size();

    EXPECT_EQ(wheels_per_side, static_cast<int>(left_count))
        << "wheels_per_side doesn't match left_wheel_names count";
    EXPECT_EQ(wheels_per_side, static_cast<int>(right_count))
        << "wheels_per_side doesn't match right_wheel_names count";
}

TEST_F(ControllerConfigTest, CmdVelTimeoutIsPositive)
{
    auto config = load_config();
    auto params = config["diff_drive_base_controller"]["ros__parameters"];

    ASSERT_TRUE(params["cmd_vel_timeout"]) << "Missing cmd_vel_timeout";
    double timeout = params["cmd_vel_timeout"].as<double>();
    EXPECT_GT(timeout, 0.0) << "cmd_vel_timeout must be positive";
}
