/// @file mobility_monitor.cpp
/// @brief Detects a bogged-down or slipping robot from commanded versus measured motion.
///
/// Perseus Lite's EKF fuses wheel odometry, so its velocity says what the wheels did,
/// not what the body did, and comparing a command against it can never show slip.
/// The monitor therefore uses two independent comparisons:
///
///   drive_linear / drive_angular  command (/cmd_vel, twist_mux's output) vs wheel
///                                 odometry: are the servos delivering what was asked?
///                                 Faults when bogged down, stalled, current-limited or
///                                 when a servo drops off the bus.
///   yaw_slip                      wheel-odometry yaw rate vs the IMU gyro: when the
///                                 wheels say "turning" and the gyro says "not much",
///                                 the wheels are spinning in sand.
///   linear_slip (optional)        wheel speed vs an independent velocity source
///                                 (reference_odom_topic), e.g. visual odometry. Off by
///                                 default because the lite has no such source yet.
///
/// Publishes perseus_interfaces/MobilityStatus on /mobility/status.

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <perseus_interfaces/msg/mobility_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <vector>

#include "perseus_monitoring/slip_detector.hpp"

namespace perseus_monitoring
{
    /// @brief A value and when it arrived.
    struct Timed
    {
        double value{0.0};
        std::optional<rclcpp::Time> stamp;
    };

    class MobilityMonitor : public rclcpp::Node
    {
    public:
        explicit MobilityMonitor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
            : rclcpp::Node("mobility_monitor", options)
        {
            max_input_age_s_ = declare_parameter<double>("max_input_age_s", 0.5);
            const double rate_hz = std::max(1.0, declare_parameter<double>("rate_hz", 10.0));

            drive_linear_ = make_channel("drive_linear", 0.05, 0.5, 1.0, 0.6);
            drive_angular_ = make_channel("drive_angular", 0.2, 0.5, 1.0, 0.6);
            yaw_slip_ = make_channel("yaw_slip", 0.2, 0.5, 1.0, 0.6);
            linear_slip_ = make_channel("linear_slip", 0.05, 0.5, 1.5, 0.8);

            const auto cmd_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
            const auto wheel_topic = declare_parameter<std::string>("wheel_odom_topic", "/diff_drive_base_controller/odom");
            const auto imu_topic = declare_parameter<std::string>("imu_topic", "/imu/data");
            const auto reference_topic = declare_parameter<std::string>("reference_odom_topic", "");
            const auto status_topic = declare_parameter<std::string>("status_topic", "/mobility/status");

            cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
                cmd_topic, 10,
                [this](const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg)
                {
                    const auto t = now();
                    cmd_linear_ = {msg->twist.linear.x, t};
                    cmd_angular_ = {msg->twist.angular.z, t};
                });
            wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                wheel_topic, 10,
                [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
                {
                    const auto t = now();
                    wheel_linear_ = {msg->twist.twist.linear.x, t};
                    wheel_angular_ = {msg->twist.twist.angular.z, t};
                });
            imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
                imu_topic, rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
                {
                    gyro_z_ = {msg->angular_velocity.z, now()};
                });
            if (!reference_topic.empty())
            {
                reference_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                    reference_topic, 10,
                    [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
                    {
                        reference_linear_ = {msg->twist.twist.linear.x, now()};
                    });
            }
            status_pub_ = create_publisher<perseus_interfaces::msg::MobilityStatus>(status_topic, 10);
            timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz), [this]()
                                       { on_timer(); });
            const std::string reference_note = reference_topic.empty() ? "" : " vs " + reference_topic;
            RCLCPP_INFO(get_logger(), "mobility_monitor: %s vs %s vs %s%s", cmd_topic.c_str(), wheel_topic.c_str(),
                        imu_topic.c_str(), reference_note.c_str());
        }

    private:
        struct Channel
        {
            std::string name;
            SlipDetector detector;
            bool was_fault{false};
        };

        Channel make_channel(const std::string& name, double min_command, double fault_ratio, double window_s,
                             double grace_s)
        {
            SlipDetectorConfig config;
            config.min_command = declare_parameter<double>(name + ".min_command", min_command);
            config.fault_ratio = declare_parameter<double>(name + ".fault_ratio", fault_ratio);
            config.window_s = declare_parameter<double>(name + ".window_s", window_s);
            config.grace_s = declare_parameter<double>(name + ".grace_s", grace_s);
            return Channel{name, SlipDetector(config)};
        }

        bool fresh(const Timed& input, const rclcpp::Time& t) const
        {
            return input.stamp && input.stamp->get_clock_type() == t.get_clock_type() &&
                   (t - *input.stamp).seconds() <= max_input_age_s_;
        }

        /// @brief Updates one channel; a stale measurement resets it, a stale command
        ///        counts as "no command" (twist_mux goes quiet when nothing drives).
        void step(Channel& channel, const Timed& commanded, const Timed& measured, const rclcpp::Time& t)
        {
            if (!fresh(measured, t))
            {
                channel.detector.reset();
                return;
            }
            const double command = fresh(commanded, t) ? commanded.value : 0.0;
            channel.detector.update(command, measured.value, t.seconds());
        }

        perseus_interfaces::msg::MobilityChannel report(Channel& channel)
        {
            perseus_interfaces::msg::MobilityChannel out;
            out.name = channel.name;
            out.commanded = channel.detector.commanded();
            out.measured = channel.detector.measured();
            out.ratio = channel.detector.ratio();
            out.evaluating = channel.detector.evaluating();
            out.fault = channel.detector.fault();
            if (out.fault != channel.was_fault)
            {
                if (out.fault)
                {
                    RCLCPP_WARN(get_logger(), "%s FAULT: commanded %.2f, measured %.2f, mean shortfall %.0f%%",
                                channel.name.c_str(), out.commanded, out.measured, 100.0 * out.ratio);
                }
                else
                {
                    RCLCPP_INFO(get_logger(), "%s recovered", channel.name.c_str());
                }
                channel.was_fault = out.fault;
            }
            return out;
        }

        void on_timer()
        {
            const rclcpp::Time t = now();
            step(drive_linear_, cmd_linear_, wheel_linear_, t);
            step(drive_angular_, cmd_angular_, wheel_angular_, t);
            // The "command" for slip is what the wheels did; ignore it unless the robot
            // is actually being driven, so a coasting wheel reading cannot trip it.
            Timed wheel_yaw = wheel_angular_;
            if (!fresh(cmd_angular_, t))
            {
                wheel_yaw.value = 0.0;
            }
            step(yaw_slip_, wheel_yaw, gyro_z_, t);

            perseus_interfaces::msg::MobilityStatus status;
            status.header.stamp = t;
            status.channels.push_back(report(drive_linear_));
            status.channels.push_back(report(drive_angular_));
            status.channels.push_back(report(yaw_slip_));
            if (reference_sub_)
            {
                Timed wheel_speed = wheel_linear_;
                if (!fresh(cmd_linear_, t))
                {
                    wheel_speed.value = 0.0;
                }
                step(linear_slip_, wheel_speed, reference_linear_, t);
                status.channels.push_back(report(linear_slip_));
            }
            status.stalled = status.channels[0].fault || status.channels[1].fault;
            status.slipping = status.channels[2].fault || (reference_sub_ && status.channels[3].fault);
            status_pub_->publish(status);
        }

        double max_input_age_s_{0.5};
        Channel drive_linear_{"drive_linear", SlipDetector(SlipDetectorConfig{})};
        Channel drive_angular_{"drive_angular", SlipDetector(SlipDetectorConfig{})};
        Channel yaw_slip_{"yaw_slip", SlipDetector(SlipDetectorConfig{})};
        Channel linear_slip_{"linear_slip", SlipDetector(SlipDetectorConfig{})};
        Timed cmd_linear_;
        Timed cmd_angular_;
        Timed wheel_linear_;
        Timed wheel_angular_;
        Timed gyro_z_;
        Timed reference_linear_;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr reference_sub_;
        rclcpp::Publisher<perseus_interfaces::msg::MobilityStatus>::SharedPtr status_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
    };

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<perseus_monitoring::MobilityMonitor>());
    rclcpp::shutdown();
    return 0;
}
