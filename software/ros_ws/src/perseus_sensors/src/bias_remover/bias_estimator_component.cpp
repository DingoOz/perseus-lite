// bias_estimator_component.cpp
// ------------------------------------------------------------
// Bias Estimator (Composable) - learns the gyro bias while the robot is
// stationary and publishes it for BiasRemover.
//
// Stationary detection (see stationary_gate.hpp):
//   * Only ENABLED sources (use_odom / use_cmd_vel) vote. A disabled source
//     never counts as "stationary".
//   * odom: stationary when |twist| < odom_threshold on every axis. No odom
//     message within odom_timeout -> unknown -> NOT stationary.
//   * cmd_vel: stationary when the last command is zero, or when no command
//     has arrived for cmd_vel_timeout (the diff-drive controller stops the
//     wheels after its own cmd_vel_timeout). Before the first message the
//     topic is unproven -> NOT stationary.
//   * stationary_mode AND / OR combines the enabled votes.
//   * Samples whose |rate| exceeds max_gyro_rate on any axis are rejected even
//     when the sources say stationary (pushed robot, stale odom, ...).
//   * If NO source is enabled the node refuses to estimate (logs an error and
//     publishes nothing): learning while possibly moving would make the
//     remover subtract a real yaw rate. BiasRemover then passes IMU data
//     through uncorrected.
//
// Nothing is published until the first stationary sample has been accepted.
// ------------------------------------------------------------

#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <stdexcept>
#include <string>

#include "bias_remover/stationary_gate.hpp"

namespace imu_processors
{
    static inline bool abslt(double v, double the) { return std::abs(v) < the; }
    static inline double ewma(double alpha, double avg, double meas)
    {
        return alpha * meas + (1.0 - alpha) * avg;
    }
    static inline bool twist_is_zero(const geometry_msgs::msg::Twist& t, double threshold)
    {
        return abslt(t.linear.x, threshold) && abslt(t.linear.y, threshold) &&
               abslt(t.linear.z, threshold) && abslt(t.angular.x, threshold) &&
               abslt(t.angular.y, threshold) && abslt(t.angular.z, threshold);
    }

    class BiasEstimator : public rclcpp::Node
    {
    public:
        explicit BiasEstimator(const rclcpp::NodeOptions& options)
            : rclcpp::Node("imu_bias_estimator", options),
              // Topics (defaults follow the perseus-lite topic contract)
              imu_in_topic_{declare_parameter<std::string>("imu_in_topic", "/imu/data")},
              bias_out_topic_{declare_parameter<std::string>("bias_out_topic", "/imu/gyro_bias")},
              cmd_vel_topic_{declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel")},
              odom_topic_{declare_parameter<std::string>("odom_topic", "/diff_drive_base_controller/odom")}
        {
            // Behavior
            const auto mode_param = declare_parameter<std::string>("stationary_mode", "AND");
            const auto mode = parse_stationary_mode(mode_param);
            if (!mode)
            {
                RCLCPP_ERROR(get_logger(), "Unknown stationary_mode '%s' (expected AND or OR); using AND",
                             mode_param.c_str());
            }
            stationary_mode_ = mode.value_or(StationaryMode::AND);

            use_cmd_vel_ = declare_parameter<bool>("use_cmd_vel", false);
            use_odom_ = declare_parameter<bool>("use_odom", true);
            // Jazzy ros2_control / twist_mux use TwistStamped on /cmd_vel
            use_stamped_ = declare_parameter<bool>("use_stamped", true);
            alpha_ = declare_parameter<double>("accumulator_alpha", 0.01);
            cmd_vel_threshold_ = declare_parameter<double>("cmd_vel_threshold", 0.001);
            odom_threshold_ = declare_parameter<double>("odom_threshold", 0.001);
            cmd_vel_timeout_ = rclcpp::Duration::from_seconds(declare_parameter<double>("cmd_vel_timeout", 0.5));
            odom_timeout_ = rclcpp::Duration::from_seconds(declare_parameter<double>("odom_timeout", 0.5));
            max_gyro_rate_ = declare_parameter<double>("max_gyro_rate", 0.1);

            if (!(alpha_ > 0.0 && alpha_ <= 1.0))
            {
                throw std::invalid_argument("accumulator_alpha must be in (0, 1]");
            }

            // Rate control
            estimator_rate_hz_ = declare_parameter<double>("estimator_rate_hz", 50.0);
            if (estimator_rate_hz_ > 0.0)
            {
                estimator_min_dt_ = rclcpp::Duration::from_seconds(1.0 / estimator_rate_hz_);
            }
            else
            {
                estimator_min_dt_ = rclcpp::Duration(0, 0);
            }
            last_estimator_tick_ = this->get_clock()->now();

            accumulator_.x = accumulator_.y = accumulator_.z = 0.0;

            if (!use_cmd_vel_ && !use_odom_)
            {
                RCLCPP_ERROR(get_logger(),
                             "Neither use_odom nor use_cmd_vel is enabled: there is no way to tell the robot is "
                             "stationary, so gyro bias will NOT be estimated (nothing is published on %s)",
                             bias_out_topic_.c_str());
                return;
            }

            bias_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(bias_out_topic_, 10);

            if (use_cmd_vel_)
            {
                if (use_stamped_)
                {
                    cmd_stamped_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
                        cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
                        std::bind(&BiasEstimator::cmd_vel_stamped_cb, this, std::placeholders::_1));
                }
                else
                {
                    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
                        cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
                        std::bind(&BiasEstimator::cmd_vel_cb, this, std::placeholders::_1));
                }
            }

            if (use_odom_)
            {
                odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                    odom_topic_, rclcpp::SystemDefaultsQoS(),
                    std::bind(&BiasEstimator::odom_cb, this, std::placeholders::_1));
            }

            imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
                imu_in_topic_, rclcpp::SensorDataQoS(),
                std::bind(&BiasEstimator::imu_cb, this, std::placeholders::_1));
        }

    private:
        void cmd_vel_cb(const geometry_msgs::msg::Twist::ConstSharedPtr& m)
        {
            twist_is_zero_ = twist_is_zero(*m, cmd_vel_threshold_);
            last_cmd_vel_ = this->get_clock()->now();
        }

        void cmd_vel_stamped_cb(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& m)
        {
            twist_is_zero_ = twist_is_zero(m->twist, cmd_vel_threshold_);
            last_cmd_vel_ = this->get_clock()->now();
        }

        void odom_cb(const nav_msgs::msg::Odometry::ConstSharedPtr& m)
        {
            odom_is_zero_ = twist_is_zero(m->twist.twist, odom_threshold_);
            last_odom_ = this->get_clock()->now();
        }

        std::optional<bool> cmd_vel_vote(const rclcpp::Time& now) const
        {
            if (!use_cmd_vel_)
                return std::nullopt;
            if (!last_cmd_vel_)
                return false;  // topic not proven to be wired up yet
            if (now - *last_cmd_vel_ > cmd_vel_timeout_)
                return true;  // command expired -> controller has stopped the wheels
            return twist_is_zero_;
        }

        std::optional<bool> odom_vote(const rclcpp::Time& now) const
        {
            if (!use_odom_)
                return std::nullopt;
            if (!last_odom_ || now - *last_odom_ > odom_timeout_)
                return false;  // no fresh odometry -> motion unknown
            return odom_is_zero_;
        }

        void imu_cb(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
        {
            const auto now = this->get_clock()->now();

            // Throttle estimator work + bias publishing
            if (estimator_rate_hz_ > 0.0 && (now - last_estimator_tick_) < estimator_min_dt_)
            {
                return;
            }
            last_estimator_tick_ = now;

            const auto& w = msg->angular_velocity;
            if (combine_stationary_votes(stationary_mode_, cmd_vel_vote(now), odom_vote(now)) &&
                gyro_within_bias_limit(w.x, w.y, w.z, max_gyro_rate_))
            {
                accumulator_.x = ewma(alpha_, accumulator_.x, w.x);
                accumulator_.y = ewma(alpha_, accumulator_.y, w.y);
                accumulator_.z = ewma(alpha_, accumulator_.z, w.z);
                have_estimate_ = true;
            }

            if (!have_estimate_)
            {
                return;
            }

            geometry_msgs::msg::Vector3Stamped bias;
            bias.header = msg->header;
            bias.vector = accumulator_;
            bias_pub_->publish(bias);
        }

    private:
        // flags
        bool twist_is_zero_{false};
        bool odom_is_zero_{false};
        bool have_estimate_{false};
        std::optional<rclcpp::Time> last_cmd_vel_;
        std::optional<rclcpp::Time> last_odom_;

        // params
        bool use_cmd_vel_{false};
        bool use_odom_{true};
        bool use_stamped_{true};
        double cmd_vel_threshold_{0.001};
        double odom_threshold_{0.001};
        double alpha_{0.01};
        double max_gyro_rate_{0.1};
        rclcpp::Duration cmd_vel_timeout_{0, 500000000};
        rclcpp::Duration odom_timeout_{0, 500000000};
        StationaryMode stationary_mode_{StationaryMode::AND};

        // rate control
        double estimator_rate_hz_{50.0};
        rclcpp::Duration estimator_min_dt_{0, 0};
        rclcpp::Time last_estimator_tick_{0, 0, RCL_ROS_TIME};

        // topics
        std::string imu_in_topic_, bias_out_topic_, cmd_vel_topic_, odom_topic_;

        // bias accumulator
        geometry_msgs::msg::Vector3 accumulator_;

        // ROS interfaces
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr bias_pub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_stamped_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    };

}  // namespace imu_processors

RCLCPP_COMPONENTS_REGISTER_NODE(imu_processors::BiasEstimator)
