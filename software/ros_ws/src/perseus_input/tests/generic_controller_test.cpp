#include "generic_controller/generic_controller.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

namespace
{
    class TestableController : public GenericController
    {
    public:
        using GenericController::GenericController;

        void feed(std::vector<float> axes, std::vector<int> buttons)
        {
            auto joy = std::make_shared<sensor_msgs::msg::Joy>();
            joy->axes = std::move(axes);
            joy->buttons = std::move(buttons);
            _joy_callback(joy);
        }
    };

    // Trigger-style deadman on axis 0 (pressed = -1), forward on axis 1, turn on
    // axis 2, and a forward button on index 14 (like the 8bitdo config).
    std::vector<rclcpp::Parameter> base_params()
    {
        return {
            rclcpp::Parameter("drive.forward.axis", 1),
            rclcpp::Parameter("drive.forward.scaling", 1.0),
            rclcpp::Parameter("drive.forward.turbo", 3.0),
            rclcpp::Parameter("drive.forward.enable.axis", 0),
            rclcpp::Parameter("drive.forward.enable.is_less_than", true),
            rclcpp::Parameter("drive.forward.enable.threshold", -0.5),
            rclcpp::Parameter("drive.forward.turbo_enable.axis", 3),
            rclcpp::Parameter("drive.forward.turbo_enable.is_less_than", true),
            rclcpp::Parameter("drive.forward.turbo_enable.threshold", -0.5),
            rclcpp::Parameter("drive.turn.axis", 2),
            rclcpp::Parameter("drive.turn.scaling", 1.0),
            rclcpp::Parameter("drive.turn.enable.follows", "drive.forward.enable"),
            rclcpp::Parameter("drive.turn.turbo_enable.follows", "drive.forward.turbo_enable"),
        };
    }

    class GenericControllerTest : public ::testing::Test
    {
    protected:
        static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
        static void TearDownTestSuite() { rclcpp::shutdown(); }

        void make(std::vector<rclcpp::Parameter> extra = {})
        {
            auto params = base_params();
            params.insert(params.end(), extra.begin(), extra.end());
            rclcpp::NodeOptions options;
            options.allow_undeclared_parameters(true);
            options.use_intra_process_comms(true);
            options.parameter_overrides(params);
            node = std::make_shared<TestableController>(options);
            sub = node->create_subscription<geometry_msgs::msg::TwistStamped>(
                "joy_vel", 10, [this](geometry_msgs::msg::TwistStamped::SharedPtr msg)
                { received.push_back(*msg); });
        }

        void spin()
        {
            for (int i = 0; i < 5; ++i)
                rclcpp::spin_some(node);
        }

        std::shared_ptr<TestableController> node;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub;
        std::vector<geometry_msgs::msg::TwistStamped> received;
    };
}  // namespace

TEST(GenericControllerHelpersTest, AxisOutOfRangeReadsZero)
{
    sensor_msgs::msg::Joy joy;
    joy.axes = {0.5f, -0.25f};
    bool bad = false;
    EXPECT_DOUBLE_EQ(GenericController::axis_or_zero(joy, 1, &bad), -0.25);
    EXPECT_FALSE(bad);
    EXPECT_DOUBLE_EQ(GenericController::axis_or_zero(joy, 7, &bad), 0.0);
    EXPECT_TRUE(bad);
    EXPECT_DOUBLE_EQ(GenericController::axis_or_zero(joy, -1, &bad), 0.0);
    EXPECT_FALSE(bad);  // negative == "not configured", not an error
}

TEST(GenericControllerHelpersTest, ButtonOutOfRangeReadsUnpressed)
{
    sensor_msgs::msg::Joy joy;
    joy.buttons = {0, 1};
    bool bad = false;
    EXPECT_TRUE(GenericController::button_or_false(joy, 1, &bad));
    EXPECT_FALSE(bad);
    EXPECT_FALSE(GenericController::button_or_false(joy, 14, &bad));
    EXPECT_TRUE(bad);
}

TEST_F(GenericControllerTest, ShortJoyMessageDoesNotCrash)
{
    // Config references button 14 and axis 7, the pad only reports 2 axes and
    // 4 buttons: this used to index past the end of the arrays (UB/segfault).
    make({rclcpp::Parameter("drive.forward.button_positive", 14),
          rclcpp::Parameter("drive.forward.enable.axis", 7)});
    node->feed({0.0f, 1.0f}, {0, 0, 0, 0});
    spin();
    EXPECT_TRUE(received.empty());  // deadman axis missing -> not enabled
}

TEST_F(GenericControllerTest, TurboIsClampedToControllerLimit)
{
    make();
    node->feed({-1.0f, 1.0f, 0.0f, -1.0f}, {});  // deadman + turbo held, full forward
    spin();
    ASSERT_EQ(received.size(), 1u);
    EXPECT_DOUBLE_EQ(received[0].twist.linear.x, 2.0);
    EXPECT_EQ(received[0].header.frame_id, "base_link");
}

TEST_F(GenericControllerTest, ReleasePublishesOneZeroThenGoesQuiet)
{
    make();
    node->feed({-1.0f, 0.5f, 0.0f, 1.0f}, {});  // deadman held
    node->feed({1.0f, 0.5f, 0.0f, 1.0f}, {});   // released
    node->feed({1.0f, 0.5f, 0.0f, 1.0f}, {});   // still released
    node->feed({1.0f, 0.5f, 0.0f, 1.0f}, {});
    spin();
    // One command, one stop, then silence so twist_mux can fall through to
    // keyboard/web/Nav2 instead of being held by a stream of zeros.
    ASSERT_EQ(received.size(), 2u);
    EXPECT_GT(received[0].twist.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(received[1].twist.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(received[1].twist.angular.z, 0.0);
}

TEST_F(GenericControllerTest, TimeoutEnableAcceptsLegacyString)
{
    EXPECT_NO_THROW(make({rclcpp::Parameter("timeout_enable", "false")}));
    EXPECT_NO_THROW(make({rclcpp::Parameter("timeout_enable", true)}));
}
