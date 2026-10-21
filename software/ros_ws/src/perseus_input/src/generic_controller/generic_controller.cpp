#include "generic_controller/generic_controller.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>

GenericController::GenericController(const rclcpp::NodeOptions& options)
    : Node("generic_controller", options)
{
    // Only the drive is handled on the lite. The perseus-v2 excavation-bucket
    // axes (lift/tilt/jaws/rotate/magnet -> /bucket_actuators) were removed:
    // the lite has no bucket, and their configs read joystick indexes (e.g.
    // axis 7, button 14) that many pads don't have.
    _axis_parsers.emplace(std::make_pair(FORWARD_BASE_NAME, AxisParser(*this, FORWARD_BASE_NAME, true)));
    _axis_parsers.emplace(std::make_pair(TURN_BASE_NAME, AxisParser(*this, TURN_BASE_NAME, true)));

    _timeout_length_ns = rcl_time_point_value_t(this->declare_parameter<int64_t>(TIMEOUT_LENGTH, 150000000));
    _max_linear = std::abs(this->declare_parameter<double>(MAX_LINEAR, DEFAULT_MAX_LINEAR));
    _max_angular = std::abs(this->declare_parameter<double>(MAX_ANGULAR, DEFAULT_MAX_ANGULAR));
    _frame_id = this->declare_parameter<std::string>(FRAME_ID, "base_link");

    _joy_subscription =
        this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&GenericController::_joy_callback, this, std::placeholders::_1));
    _twist_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("joy_vel", 10);
    if (_read_timeout_enable())
    {
        _joy_timeout_timer = this->create_wall_timer(JOY_TIMEOUT, std::bind(&GenericController::_joy_timeout_callback, this));
    }
    _prev_received_joy_time = this->now();
    RCLCPP_INFO(this->get_logger(), "Generic controller initialized (max %.2f m/s, %.2f rad/s)", _max_linear, _max_angular);
}

bool GenericController::_read_timeout_enable()
{
    // Declared dynamically typed for backwards compatibility: launch files now
    // pass a real bool, but older launch files / YAML / CLI overrides passed
    // the string "true"/"false".
    rcl_interfaces::msg::ParameterDescriptor descriptor{};
    descriptor.description = "Publish a single zero twist when /joy goes stale (bool)";
    descriptor.dynamic_typing = true;
    const auto value = this->declare_parameter(TIMEOUT_ENABLE, rclcpp::ParameterValue(true), descriptor);
    switch (value.get_type())
    {
    case rclcpp::ParameterType::PARAMETER_BOOL:
        return value.get<bool>();
    case rclcpp::ParameterType::PARAMETER_INTEGER:
        return value.get<int64_t>() != 0;
    case rclcpp::ParameterType::PARAMETER_STRING:
    {
        std::string text = value.get<std::string>();
        std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c)
                       { return std::tolower(c); });
        return !(text == "false" || text == "0" || text == "no" || text == "off");
    }
    default:
        return true;
    }
}

double GenericController::axis_or_zero(const sensor_msgs::msg::Joy& joy, int index, bool* out_of_range)
{
    const bool bad = index >= 0 && static_cast<size_t>(index) >= joy.axes.size();
    if (out_of_range)
        *out_of_range = bad;
    if (index < 0 || bad)
        return 0.0;
    return joy.axes[static_cast<size_t>(index)];
}

bool GenericController::button_or_false(const sensor_msgs::msg::Joy& joy, int index, bool* out_of_range)
{
    const bool bad = index >= 0 && static_cast<size_t>(index) >= joy.buttons.size();
    if (out_of_range)
        *out_of_range = bad;
    if (index < 0 || bad)
        return false;
    return joy.buttons[static_cast<size_t>(index)] != 0;
}

void GenericController::_publish_twist(double forward, double turn)
{
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = _frame_id;
    // Clamp to the diff_drive_base_controller limits (turbo can exceed them).
    twist_msg.twist.linear.x = std::clamp(forward, -_max_linear, _max_linear);
    twist_msg.twist.angular.z = std::clamp(turn, -_max_angular, _max_angular);
    _twist_publisher->publish(twist_msg);
}

bool GenericController::_drive_enabled()
{
    for (const auto& base : {FORWARD_BASE_NAME, TURN_BASE_NAME})
    {
        if (_enable_parsers.at(base + ".enable").get_value() ||
            _enable_parsers.at(base + ".turbo_enable").get_value())
            return true;
    }
    return false;
}

void GenericController::_joy_timeout_callback(void)
{
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Timeout length is %ld ns", _timeout_length_ns);
    if ((this->now().nanoseconds() - _prev_received_joy_time.nanoseconds()) > _timeout_length_ns)
    {
        if (_commanding)
        {
            // One zero to stop the robot, then stay quiet so twist_mux can
            // fall through to lower-priority sources.
            _publish_twist(0.0, 0.0);
            _commanding = false;
        }
        // Warn user if controller is disconnected
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Joy timeout: no /joy for > %ld ns, drive command zeroed", _timeout_length_ns);
    }
}

void GenericController::_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    _last_received_joy = msg;
    _prev_received_joy_time = this->now();
    const bool enabled = _drive_enabled();
    double forward = _axis_parsers.at(FORWARD_BASE_NAME).get_value();
    double turn = _axis_parsers.at(TURN_BASE_NAME).get_value();

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Forward: %+2.2f, Turn: %+2.2f, Enabled: %d", forward, turn, enabled);

    if (enabled)
    {
        _publish_twist(forward, turn);
        _commanding = true;
    }
    else if (_commanding)
    {
        // Deadman released: stop once, then go quiet (see _commanding).
        _publish_twist(0.0, 0.0);
        _commanding = false;
    }
}

GenericController::AxisParser::AxisParser(GenericController& parent, const std::string& param_base_name, bool has_enable)
    : _parent(parent),
      _param_base_name(param_base_name),
      _has_enable(has_enable)
{
    using namespace rcl_interfaces::msg;

    ParameterDescriptor axis_descriptor{};
    axis_descriptor.type = ParameterType::PARAMETER_INTEGER;
    axis_descriptor.description = "The analog axis to use for parsing";

    ParameterDescriptor button_positive_descriptor{};
    button_positive_descriptor.type = ParameterType::PARAMETER_INTEGER;
    button_positive_descriptor.description = "The button to use which will set the axis to its maximum";

    ParameterDescriptor button_negative_descriptor{};
    button_negative_descriptor.type = ParameterType::PARAMETER_INTEGER;
    button_negative_descriptor.description = "The button to use which will set the axis to its minimum";

    _int_param_map = {
        {"axis",
         {-1, axis_descriptor}},
        {"button_positive",
         {-1, button_positive_descriptor}},
        {"button_negative",
         {-1, button_negative_descriptor}},
    };

    ParameterDescriptor scaling_descriptor{};
    scaling_descriptor.type = ParameterType::PARAMETER_DOUBLE;
    scaling_descriptor.description = "The axis output scale";

    ParameterDescriptor deadband_descriptor{};
    deadband_descriptor.type = ParameterType::PARAMETER_DOUBLE;
    deadband_descriptor.description = "The deadband range for the axis. If positive is NaN, no deadband is applied, and if negative is NaN, it default to negative positive deadband";

    ParameterDescriptor turbo_descriptor{};
    turbo_descriptor.type = ParameterType::PARAMETER_DOUBLE;
    turbo_descriptor.description = "Turbo scaling value";

    _double_param_map = {{"scaling",
                          {std::numeric_limits<double>::quiet_NaN(), scaling_descriptor}},
                         {"turbo",
                          {std::numeric_limits<double>::quiet_NaN(), turbo_descriptor}},
                         {"deadband_positive",
                          {0.08, deadband_descriptor}},
                         {"deadband_negative",
                          {std::numeric_limits<double>::quiet_NaN(), deadband_descriptor}}};

    ParameterDescriptor follows_descriptor{};
    follows_descriptor.type = ParameterType::PARAMETER_STRING;
    follows_descriptor.description = "The enable configuration to base initial parameters off of";
    _string_param_map = {{"follows",
                          {"", follows_descriptor}}};

    _parent.declare_parameters(param_base_name, _int_param_map);
    _parent.declare_parameters(param_base_name, _double_param_map);
    _parent.declare_parameters(param_base_name, _string_param_map);

    ParameterDescriptor hold_descriptor{};
    hold_descriptor.type = ParameterType::PARAMETER_BOOL;
    hold_descriptor.description = "Whether or not to hold the previous value if no input received";
    _parent.declare_parameter(param_base_name + ".hold", false, hold_descriptor);

    if (has_enable)
    {
        std::string enable_name = param_base_name + ".enable";
        std::string turbo_name = param_base_name + ".turbo_enable";
        _parent._enable_parsers.emplace(std::make_pair(enable_name, EnableParser(_parent, enable_name)));
        _parent._enable_parsers.emplace(std::make_pair(turbo_name, EnableParser(_parent, turbo_name)));
    }
}

double GenericController::AxisParser::get_value()
{
    if (_has_enable &&
        !_parent._enable_parsers.at(_param_base_name + ".enable").get_value() &&
        !_parent._enable_parsers.at(_param_base_name + ".turbo_enable").get_value())
        return 0.0;

    auto params = _resolve_params();

    double axis_value = 0.0;
    int axis_idx = params[AXIS_IDX].as_int();
    bool has_analog_axis = axis_idx >= 0;
    bool hold = _parent.get_parameter(_param_base_name + ".hold").as_bool();
    if (!_parent._last_received_joy)
        return 0.0;
    const auto& joy = *_parent._last_received_joy;
    if (has_analog_axis)
    {
        bool out_of_range = false;
        axis_value = axis_or_zero(joy, axis_idx, &out_of_range);
        if (out_of_range)
            _warn_out_of_range("axis", axis_idx);
    }

    double scaling = params[SCALING_IDX].as_double();
    if (_has_enable && _parent._enable_parsers.at(_param_base_name + ".turbo_enable").get_value())
    {
        scaling = params[TURBO_IDX].as_double();
        if (!std::isfinite(scaling))
            scaling = 2.0;
    }
    if (!std::isfinite(scaling))
        scaling = 1.0;

    double deadband_positive = params[DEADBAND_POSITIVE_IDX].as_double();
    double deadband_negative = params[DEADBAND_NEGATIVE_IDX].as_double();
    if (!std::isfinite(deadband_negative))
        deadband_negative = -deadband_positive;

    if (std::isfinite(deadband_positive))
    {
        bool clamped = axis_value < deadband_positive && axis_value > deadband_negative;
        if (clamped)
            axis_value = 0.0;
        else if (axis_value > deadband_positive)
            axis_value -= deadband_positive;
        else if (axis_value < deadband_negative)
            axis_value -= deadband_negative;

        // re-normalize the axis value
        if (!clamped)
        {
            if (axis_value > 0.0)
                axis_value /= (1.0 - std::abs(deadband_positive));
            else
                axis_value /= (1.0 - std::abs(deadband_negative));
        }
    }

    bool out_of_range = false;
    const int button_positive_idx = params[BUTTON_POSITIVE_IDX].as_int();
    bool button_positive = button_or_false(joy, button_positive_idx, &out_of_range);
    if (out_of_range)
        _warn_out_of_range("button", button_positive_idx);

    const int button_negative_idx = params[BUTTON_NEGATIVE_IDX].as_int();
    bool button_negative = button_or_false(joy, button_negative_idx, &out_of_range);
    if (out_of_range)
        _warn_out_of_range("button", button_negative_idx);

    bool both_buttons = button_positive && button_negative;
    if (!both_buttons)
    {
        if (button_positive)
            axis_value = 1.0;
        if (button_negative)
            axis_value = -1.0;
    }
    else
        axis_value = 0.0;

    axis_value *= scaling;
    bool should_update_analog_axis = has_analog_axis && (!hold || (std::abs(axis_value) > 0));
    if (button_positive || button_negative || should_update_analog_axis)
        _last_value = axis_value;
    else if (hold)
        return _last_value;

    return axis_value;
}

void GenericController::AxisParser::_warn_out_of_range(const char* kind, int index) const
{
    // Out-of-range indexes read as 0 / unpressed (a config written for a pad
    // with more axes/buttons than the connected one must not crash the node).
    RCLCPP_WARN_THROTTLE(_parent.get_logger(), *_parent.get_clock(), 5000,
                         "%s: %s index %d is out of range for the connected joystick (%zu axes, %zu buttons); treating it as 0/unpressed",
                         _param_base_name.c_str(), kind, index,
                         _parent._last_received_joy ? _parent._last_received_joy->axes.size() : size_t{0},
                         _parent._last_received_joy ? _parent._last_received_joy->buttons.size() : size_t{0});
}

std::vector<rclcpp::Parameter> GenericController::AxisParser::_resolve_params()
{
    // TODO: Memoize this with invalidation on parameter changes
    auto params = _parent.get_parameters(_get_param_names(_param_base_name));
    // has an axis parser with that key
    if (auto followed = _parent._axis_parsers.find(params[FOLLOWS_IDX].as_string());
        followed != _parent._axis_parsers.end())
    {
        auto followed_params = followed->second._resolve_params();
        if (auto axis = followed_params[AXIS_IDX]; axis.as_int() >= 0)
            params[AXIS_IDX] = axis;
        if (auto button_positive = followed_params[BUTTON_POSITIVE_IDX]; button_positive.as_int() >= 0)
            params[BUTTON_POSITIVE_IDX] = button_positive;
        if (auto button_negative = followed_params[BUTTON_NEGATIVE_IDX]; button_negative.as_int() >= 0)
            params[BUTTON_NEGATIVE_IDX] = button_negative;
        if (auto scaling = followed_params[SCALING_IDX]; std::isfinite(scaling.as_double()))
            params[SCALING_IDX] = scaling;
        if (auto deadband_positive = followed_params[DEADBAND_POSITIVE_IDX]; std::isfinite(deadband_positive.as_double()))
            params[DEADBAND_POSITIVE_IDX] = deadband_positive;
        if (auto deadband_negative = followed_params[DEADBAND_NEGATIVE_IDX]; std::isfinite(deadband_negative.as_double()))
            params[DEADBAND_NEGATIVE_IDX] = deadband_negative;
        if (auto turbo = followed_params[TURBO_IDX]; std::isfinite(turbo.as_double()))
            params[TURBO_IDX] = turbo;
    }

    return params;
}

std::vector<std::string> GenericController::AxisParser::_get_param_names(const std::string& base_name)
{
    return {base_name + ".axis",
            base_name + ".button_positive",
            base_name + ".button_negative",
            base_name + ".scaling",
            base_name + ".follows",
            base_name + ".deadband_positive",
            base_name + ".deadband_negative",
            base_name + ".turbo"};
}

bool GenericController::AxisParser::_parent_has_all_params(const std::string& follows_name)
{
    for (const auto& param : _get_param_names(follows_name))
        if (!_parent.has_parameter(param))
            return false;

    return true;
}

bool GenericController::AxisParser::_parent_has_any_params(const std::string& follows_name)
{
    for (const auto& param : _get_param_names(follows_name))
    {
        if (_parent.has_parameter(param))
            return true;
    }

    return false;
}

GenericController::EnableParser::EnableParser(GenericController& parent,
                                              const std::string& param_base_name,
                                              bool is_turbo)
    : _parent(parent),
      _param_base_name(param_base_name)
{
    (void)is_turbo;
    using namespace rcl_interfaces::msg;

    auto [vals, was_inserted] = _parent._axis_parsers.emplace(
        std::make_pair(param_base_name, AxisParser(parent, param_base_name)));
    if (!was_inserted)
    {
        throw std::runtime_error("Failed to insert axis parser");
    }

    ParameterDescriptor threshold_descriptor{};
    threshold_descriptor.type = ParameterType::PARAMETER_DOUBLE;
    threshold_descriptor.description = "The threshold to compare against";
    _parent.declare_parameter(param_base_name + ".threshold", std::numeric_limits<double>::quiet_NaN(), threshold_descriptor);

    ParameterDescriptor is_less_than_descriptor{};
    is_less_than_descriptor.type = ParameterType::PARAMETER_BOOL;
    is_less_than_descriptor.description = "Whether or not to enable when the axis value is less than the threshold";
    _parent.declare_parameter(param_base_name + ".is_less_than", false, threshold_descriptor);

    // if (is_turbo)
    // {
    //     ParameterDescriptor turbo_descriptor{};
    //     turbo_descriptor.type = ParameterType::PARAMETER_DOUBLE;
    //     turbo_descriptor.description = "";
    //     _parent.declare_parameter(param_base_name + ".turbo_speed", 2.0, turbo_descriptor);
    // }
}

std::vector<rclcpp::Parameter> GenericController::EnableParser::_resolve_params()
{
    // TODO: Memoize this with invalidation on parameter changes
    auto params = _parent.get_parameters(_get_param_names(_param_base_name));
    // has an axis parser with that key
    if (auto followed = _parent._enable_parsers.find(params[FOLLOWS_IDX].as_string());
        followed != _parent._enable_parsers.end())
    {
        auto followed_params = followed->second._resolve_params();
        if (auto threshold = followed_params[THRESHOLD_IDX]; std::isfinite(threshold.as_double()))
        {
            params[THRESHOLD_IDX] = threshold;
            params[IS_LT_IDX] = followed_params[IS_LT_IDX];
        }
    }

    return params;
}

std::vector<std::string> GenericController::EnableParser::_get_param_names(const std::string& base_name)
{
    return {base_name + ".follows",
            base_name + ".threshold",
            base_name + ".is_less_than"};
}

bool GenericController::EnableParser::get_value()
{
    auto params = _resolve_params();

    double threshold = params[THRESHOLD_IDX].as_double();
    if (!std::isfinite(threshold))
        threshold = 0.5;

    bool is_less_than = params[IS_LT_IDX].as_bool();
    double axis_value = _parent._axis_parsers.at(_param_base_name).get_value();
    if (is_less_than)
        return axis_value < threshold;
    return axis_value > threshold;
}
