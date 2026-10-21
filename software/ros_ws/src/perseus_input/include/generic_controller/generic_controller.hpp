#pragma once

#include <chrono>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string>

class GenericController : public rclcpp::Node
{
public:
    explicit GenericController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /// Return joy->axes[index], or 0.0 if the index is negative or out of range.
    static double axis_or_zero(const sensor_msgs::msg::Joy& joy, int index, bool* out_of_range = nullptr);
    /// Return joy->buttons[index] != 0, or false if the index is negative or out of range.
    static bool button_or_false(const sensor_msgs::msg::Joy& joy, int index, bool* out_of_range = nullptr);

private:
    friend class AxisParser;
    friend class EnableParser;

    class EnableParser;
    class AxisParser
    {
    public:
        AxisParser(GenericController& parent, const std::string& param_base_name, bool has_enable = false);

        double get_value();

    private:
        std::vector<rclcpp::Parameter> _resolve_params();
        static std::vector<std::string> _get_param_names(const std::string& base_name);
        bool _parent_has_all_params(const std::string& follows_name);
        bool _parent_has_any_params(const std::string& follows_name);
        void _warn_out_of_range(const char* kind, int index) const;

        static constexpr size_t AXIS_IDX = 0;
        static constexpr size_t BUTTON_POSITIVE_IDX = 1;
        static constexpr size_t BUTTON_NEGATIVE_IDX = 2;
        static constexpr size_t SCALING_IDX = 3;
        static constexpr size_t FOLLOWS_IDX = 4;
        static constexpr size_t DEADBAND_POSITIVE_IDX = 5;
        static constexpr size_t DEADBAND_NEGATIVE_IDX = 6;
        static constexpr size_t TURBO_IDX = 7;

        std::map<std::string, std::pair<int, rcl_interfaces::msg::ParameterDescriptor>>
            _int_param_map;
        std::map<std::string, std::pair<double, rcl_interfaces::msg::ParameterDescriptor>>
            _double_param_map;
        std::map<std::string, std::pair<std::string, rcl_interfaces::msg::ParameterDescriptor>>
            _string_param_map;

        GenericController& _parent;
        const std::string _param_base_name;
        bool _has_enable;
        double _last_value = 0.0;
    };

    class EnableParser
    {
    public:
        EnableParser(GenericController& parent, const std::string& param_base_name, bool is_turbo = false);

        bool get_value();

    private:
        std::vector<rclcpp::Parameter> _resolve_params();
        static std::vector<std::string> _get_param_names(const std::string& base_name);

        static constexpr size_t FOLLOWS_IDX = 0;
        static constexpr size_t THRESHOLD_IDX = 1;
        static constexpr size_t IS_LT_IDX = 2;

        GenericController& _parent;
        const std::string _param_base_name;
    };
    void _publish_twist(double forward, double turn);
    bool _drive_enabled();
    bool _read_timeout_enable();

    static constexpr std::string FORWARD_BASE_NAME = "drive.forward";
    static constexpr std::string TURN_BASE_NAME = "drive.turn";
    static constexpr std::string TIMEOUT_LENGTH = "timeout_ns";
    static constexpr std::string TIMEOUT_ENABLE = "timeout_enable";
    static constexpr const char* MAX_LINEAR = "max_linear_velocity";
    static constexpr const char* MAX_ANGULAR = "max_angular_velocity";
    static constexpr std::string FRAME_ID = "frame_id";

    // Defaults match diff_drive_base_controller's limits in
    // perseus_lite/config/perseus_lite_controllers.yaml (linear.x 2.0 m/s,
    // angular.z 5.0 rad/s): never command more than the controller accepts.
    static constexpr double DEFAULT_MAX_LINEAR = 2.0;
    static constexpr double DEFAULT_MAX_ANGULAR = 5.0;

    static constexpr auto JOY_TIMEOUT = std::chrono::milliseconds(100);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_subscription;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr _twist_publisher;
    rclcpp::TimerBase::SharedPtr _joy_timeout_timer;
    rcl_time_point_value_t _timeout_length_ns = 150000000;
    double _max_linear = DEFAULT_MAX_LINEAR;
    double _max_angular = DEFAULT_MAX_ANGULAR;
    std::string _frame_id = "base_link";
    // True while a deadman is held (we are actively commanding). Used so the
    // node publishes a single zero on release/timeout and then goes quiet,
    // instead of streaming zeros on /joy_vel that would hold twist_mux's
    // highest-priority input and starve keyboard/web/Nav2 forever.
    bool _commanding = false;

protected:
    // Protected (not private) so unit tests can drive them directly.
    void _joy_timeout_callback(void);
    void _joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    sensor_msgs::msg::Joy::SharedPtr _last_received_joy;
    rclcpp::Time _prev_received_joy_time;
    std::map<std::string, AxisParser> _axis_parsers;
    std::map<std::string, EnableParser> _enable_parsers;
};
