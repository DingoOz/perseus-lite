#include "st3215_system.hpp"

#include <termios.h>

#include <algorithm>
#include <cmath>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "st3215_protocol.hpp"

namespace
{
    template <typename T>
    bool has_parameter(const std::unordered_map<std::string, T>& params, const std::string& key)
    {
        return params.find(key) != params.end();
    }
}

namespace perseus_lite_hardware
{
    namespace
    {
        static const char* const LOGGER_NAME = "ST3215SystemHardware";

        // Throttled logging from the communication thread must not depend on
        // the controller manager's clock type.
        rclcpp::Clock& steady_log_clock()
        {
            static rclcpp::Clock clock(RCL_STEADY_TIME);
            return clock;
        }
    }  // namespace

    ST3215SystemHardware::~ST3215SystemHardware()
    {
        if (_comm_thread_running)
        {
            auto deactivate_result = on_deactivate(rclcpp_lifecycle::State());
            if (deactivate_result != hardware_interface::CallbackReturn::SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Failed to deactivate");
            }
        }

        auto cleanup_result = on_cleanup(rclcpp_lifecycle::State());
        if (cleanup_result != hardware_interface::CallbackReturn::SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Failed to cleanup");
        }
    }

    hardware_interface::CallbackReturn ST3215SystemHardware::on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params)
    {
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Initializing ST3215 hardware interface with %zu joints", params.hardware_info.joints.size());
        if (const auto result = SystemInterface::on_init(params);
            result != hardware_interface::CallbackReturn::SUCCESS)
        {
            return result;
        }

        auto logger = rclcpp::get_logger(LOGGER_NAME);
        const auto& hardware_parameters = params.hardware_info.hardware_parameters;

        // Check required parameters
        if (!has_parameter(hardware_parameters, "serial_port"))
        {
            RCLCPP_ERROR(logger, "Missing required parameter 'serial_port'");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (!has_parameter(hardware_parameters, "baud_rate"))
        {
            RCLCPP_ERROR(logger, "Missing required parameter 'baud_rate'");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_DEBUG(logger, "Serial configuration - Port: %s, Baud Rate: %s",
                     hardware_parameters.at("serial_port").c_str(),
                     hardware_parameters.at("baud_rate").c_str());

        // Speed scale: the RPM that a full-scale (32767) speed value represents
        if (has_parameter(hardware_parameters, "servo_max_rpm"))
        {
            try
            {
                _servo_max_rpm = std::stod(hardware_parameters.at("servo_max_rpm"));
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(logger, "Invalid 'servo_max_rpm': %s", e.what());
                return hardware_interface::CallbackReturn::ERROR;
            }
            if (!std::isfinite(_servo_max_rpm) || _servo_max_rpm <= 0.0)
            {
                RCLCPP_ERROR(logger, "'servo_max_rpm' must be positive, got %f", _servo_max_rpm);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        RCLCPP_INFO(logger, "Using servo_max_rpm: %.1f", _servo_max_rpm);

        if (has_parameter(hardware_parameters, "acceleration"))
        {
            try
            {
                const int acceleration = std::stoi(hardware_parameters.at("acceleration"));
                if (acceleration < 0 || acceleration > 254)
                {
                    RCLCPP_ERROR(logger, "'acceleration' must be 0-254, got %d", acceleration);
                    return hardware_interface::CallbackReturn::ERROR;
                }
                _acceleration = static_cast<uint8_t>(acceleration);
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(logger, "Invalid 'acceleration': %s", e.what());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        // Pre-allocate vectors with known size
        const auto joint_count = params.hardware_info.joints.size();
        _command_speeds.assign(joint_count, 0.0);
        _current_positions.assign(joint_count, 0.0);
        _current_velocities.assign(joint_count, 0.0);
        _temperatures.assign(joint_count, _ROOM_TEMPERATURE_CELSIUS);  // Initialize to room temperature
        _servo_ids.clear();
        _servo_ids.reserve(joint_count);

        _servo_states.assign(joint_count, ServoState{});

        // Servos whose drive direction is inverted (left side of the rover).
        // Override with the hardware parameter 'inverted_servo_ids', a
        // comma-separated ID list, when the wiring differs.
        _inverted_servo_ids = {2, 3};
        if (has_parameter(hardware_parameters, "inverted_servo_ids"))
        {
            _inverted_servo_ids.clear();
            std::stringstream ids_ss(hardware_parameters.at("inverted_servo_ids"));
            std::string token;
            while (std::getline(ids_ss, token, ','))
            {
                try
                {
                    _inverted_servo_ids.push_back(static_cast<uint8_t>(std::stoi(token)));
                }
                catch (const std::exception& e)
                {
                    RCLCPP_ERROR(logger, "Invalid 'inverted_servo_ids' entry '%s': %s",
                                 token.c_str(), e.what());
                    return hardware_interface::CallbackReturn::ERROR;
                }
            }
        }

        // Extract and validate servo IDs
        for (const auto& joint : params.hardware_info.joints)
        {
            if (!has_parameter(joint.parameters, "id"))
            {
                RCLCPP_ERROR(logger, "Joint '%s' is missing required parameter 'id'",
                             joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }

            try
            {
                const auto id = static_cast<uint8_t>(
                    std::stoi(joint.parameters.at("id")));
                _servo_ids.push_back(id);
            }
            catch (const std::exception& e)
            {
                RCLCPP_FATAL(logger,
                             "Failed to parse ID for joint '%s': %s",
                             joint.name.c_str(), e.what());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        // Verify command interfaces
        for (const auto& joint : params.hardware_info.joints)
        {
            if (!verify_command_interfaces(joint, logger))
            {
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> ST3215SystemHardware::export_state_interfaces()
    {
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Exporting state interfaces");

        std::vector<hardware_interface::StateInterface> state_interfaces;
        const auto& info = get_hardware_info();

        for (size_t i = 0; i < info.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info.joints[i].name, hardware_interface::HW_IF_POSITION, &_current_positions[i]));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_current_velocities[i]));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info.joints[i].name, hardware_interface::HW_IF_TEMPERATURE, &_temperatures[i]));
        }

        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Exported %zu state interfaces", state_interfaces.size());
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ST3215SystemHardware::export_command_interfaces()
    {
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Exporting command interfaces");

        std::vector<hardware_interface::CommandInterface> command_interfaces;
        const auto& info = get_hardware_info();

        for (size_t i = 0; i < info.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_command_speeds[i]));
        }

        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Exported %zu command interfaces", command_interfaces.size());
        return command_interfaces;
    }

    hardware_interface::CallbackReturn ST3215SystemHardware::on_configure(
        const rclcpp_lifecycle::State&)
    {
        auto logger = rclcpp::get_logger(LOGGER_NAME);
        try
        {
            const auto& serial_port = info_.hardware_parameters.at("serial_port");
            const int baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));

            if (_serial_port.is_open())
            {
                _serial_port.close();
            }
            _serial_port.open(serial_port);

            // Configure serial port settings
            _serial_port.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
            _serial_port.set_option(boost::asio::serial_port_base::character_size(8));
            _serial_port.set_option(boost::asio::serial_port_base::stop_bits(
                boost::asio::serial_port_base::stop_bits::one));
            _serial_port.set_option(boost::asio::serial_port_base::parity(
                boost::asio::serial_port_base::parity::none));
            _serial_port.set_option(boost::asio::serial_port_base::flow_control(
                boost::asio::serial_port_base::flow_control::none));

            {
                std::lock_guard<std::mutex> state_lock(_state_mutex);
                std::fill(_servo_states.begin(), _servo_states.end(), ServoState{});
            }

            const uint8_t goal_speed_register = static_cast<uint8_t>(ServoSramRegister::GOAL_SPEED_L);
            const uint8_t mode_register = static_cast<uint8_t>(ServoEpromRegister::MODE);
            const uint8_t acc_register = static_cast<uint8_t>(ServoSramRegister::ACC);
            const uint8_t torque_register = static_cast<uint8_t>(ServoSramRegister::TORQUE_ENABLE);

            for (uint8_t servo_id : _servo_ids)
            {
                // Zero the goal speed first: a servo left spinning by a previous
                // crashed run must not keep moving once torque is (re)enabled.
                RCLCPP_DEBUG(logger, "Zeroing goal speed for servo %d", servo_id);
                if (!send_servo_command(servo_id, ServoCommand::WRITE, std::array<uint8_t, 3>{goal_speed_register, 0, 0}))
                {
                    RCLCPP_ERROR(logger, "Failed to zero goal speed for servo %d", servo_id);
                    _serial_port.close();
                    return hardware_interface::CallbackReturn::ERROR;
                }
                std::this_thread::sleep_for(_COMMAND_DELAY);

                RCLCPP_DEBUG(logger, "Setting wheel mode for servo %d", servo_id);
                if (!send_servo_command(servo_id, ServoCommand::WRITE, std::array<uint8_t, 2>{mode_register, _WHEEL_MODE_VALUE}))
                {
                    RCLCPP_ERROR(logger, "Failed to set wheel mode for servo %d", servo_id);
                    _serial_port.close();
                    return hardware_interface::CallbackReturn::ERROR;
                }
                std::this_thread::sleep_for(_COMMAND_DELAY);

                RCLCPP_DEBUG(logger, "Setting acceleration %d for servo %d", _acceleration, servo_id);
                if (!send_servo_command(servo_id, ServoCommand::WRITE, std::array<uint8_t, 2>{acc_register, _acceleration}))
                {
                    RCLCPP_ERROR(logger, "Failed to set acceleration for servo %d", servo_id);
                    _serial_port.close();
                    return hardware_interface::CallbackReturn::ERROR;
                }
                std::this_thread::sleep_for(_COMMAND_DELAY);

                RCLCPP_DEBUG(logger, "Enabling torque for servo %d", servo_id);
                if (!send_servo_command(servo_id, ServoCommand::WRITE, std::array<uint8_t, 2>{torque_register, _TORQUE_ENABLE_VALUE}))
                {
                    RCLCPP_ERROR(logger, "Failed to enable torque for servo %d", servo_id);
                    _serial_port.close();
                    return hardware_interface::CallbackReturn::ERROR;
                }
                std::this_thread::sleep_for(_COMMAND_DELAY);
            }

            // Writes are fire-and-forget, so confirm each servo actually answers.
            // A missing servo is reported loudly but does not block bringup; its
            // feedback stays stale (zero velocity) until it responds.
            size_t responding = 0;
            for (size_t i = 0; i < _servo_ids.size(); ++i)
            {
                bool ok = false;
                for (int attempt = 0; attempt < _STARTUP_PROBE_ATTEMPTS && !ok; ++attempt)
                {
                    std::lock_guard<std::mutex> lock(_serial_mutex);
                    ok = request_status(i);
                }
                if (ok)
                {
                    ++responding;
                }
                else
                {
                    RCLCPP_ERROR(logger, "Servo %d did not respond to a status request - check wiring, ID and baud rate",
                                 _servo_ids[i]);
                }
            }
            RCLCPP_INFO(logger, "Configured %zu servos in wheel mode with torque enabled (%zu responding)",
                        _servo_ids.size(), responding);

            // Start communication thread
            _comm_thread_running = true;
            _comm_thread = std::thread(&ST3215SystemHardware::communication_thread, this);

            RCLCPP_INFO(logger, "Successfully configured!");
            return hardware_interface::CallbackReturn::SUCCESS;
        }
        catch (const std::exception& e)
        {
            // boost::system::system_error from the port, std::invalid_argument /
            // std::out_of_range from a malformed baud_rate
            RCLCPP_FATAL(logger, "Failed to configure serial port: %s", e.what());
            boost::system::error_code ignored;
            _serial_port.close(ignored);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    void ST3215SystemHardware::communication_thread() noexcept
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Starting communication thread");

        while (_comm_thread_running)
        {
            // Request status from each servo in sequence
            for (size_t i = 0; i < _servo_ids.size() && _comm_thread_running; ++i)
            {
                // Hold the serial mutex across the whole request/response
                // transaction so a velocity write() from the controller
                // thread cannot interleave between request and reply.
                std::lock_guard<std::mutex> lock(_serial_mutex);
                if (!request_status(i))
                {
                    RCLCPP_WARN_THROTTLE(rclcpp::get_logger(LOGGER_NAME),
                                         steady_log_clock(), 1000,
                                         "No status reply from servo %d", _servo_ids[i]);
                }
            }

            // Wait before starting next update cycle
            std::this_thread::sleep_for(_COMMUNICATION_CYCLE_DELAY);
        }

        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Communication thread stopped");
    }

    bool ST3215SystemHardware::request_status(const size_t index) noexcept
    {
        const auto servo_id = _servo_ids[index];

        // READ(0x02) from the position register (0x38): position(2), speed(2),
        // load(2), voltage(1), temperature(1)
        const std::array<uint8_t, 2> read_data{
            _PRESENT_POSITION_REG,
            static_cast<uint8_t>(protocol::STATUS_DATA_SIZE)};

        try
        {
            discard_pending_input();
            if (!send_servo_command(servo_id, ServoCommand::READ, std::span{read_data}))
            {
                return false;
            }

            // Accumulate until a complete status packet from this servo arrives
            // or the deadline passes; a reply can be split across several reads.
            std::array<uint8_t, _BUFFER_SIZE> buffer{};
            size_t total = 0;
            const auto deadline = std::chrono::steady_clock::now() + _RESPONSE_TIMEOUT;
            while (total < buffer.size())
            {
                const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                    deadline - std::chrono::steady_clock::now());
                if (remaining.count() <= 0)
                {
                    return false;
                }

                boost::system::error_code error;
                const size_t bytes_read = read_with_timeout(
                    std::span{buffer.data() + total, buffer.size() - total}, remaining, error);
                if (error)
                {
                    if (error != boost::asio::error::operation_aborted)
                    {
                        RCLCPP_WARN_THROTTLE(rclcpp::get_logger(LOGGER_NAME),
                                             steady_log_clock(), 1000,
                                             "Error reading from servo %d: %s",
                                             servo_id, error.message().c_str());
                    }
                    return false;
                }

                total += bytes_read;
                if (process_response(std::span{buffer.data(), total}, servo_id))
                {
                    return true;
                }
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR_THROTTLE(rclcpp::get_logger(LOGGER_NAME), steady_log_clock(), 1000,
                                  "Serial communication error: %s", e.what());
        }
        return false;
    }

    size_t ST3215SystemHardware::read_with_timeout(std::span<uint8_t> buffer,
                                                   std::chrono::milliseconds timeout,
                                                   boost::system::error_code& error)
    {
        size_t bytes_read = 0;
        error = boost::asio::error::operation_aborted;

        boost::asio::steady_timer deadline(_io_context, timeout);

        _serial_port.async_read_some(
            boost::asio::buffer(buffer.data(), buffer.size()),
            [&](const boost::system::error_code& ec, size_t n)
            {
                error = ec;
                bytes_read = n;
                deadline.cancel();
            });

        deadline.async_wait(
            [&](const boost::system::error_code& ec)
            {
                // Timer fired (not cancelled by a completed read): abort the read
                if (ec != boost::asio::error::operation_aborted)
                {
                    _serial_port.cancel();
                }
            });

        _io_context.restart();
        _io_context.run();

        return error ? 0 : bytes_read;
    }

    void ST3215SystemHardware::discard_pending_input() noexcept
    {
        // Servos acknowledge every WRITE; those replies (and any line noise)
        // must not be mistaken for the status reply that follows.
        if (_serial_port.is_open())
        {
            ::tcflush(_serial_port.native_handle(), TCIFLUSH);
        }
    }

    void ST3215SystemHardware::stop_communication_thread() noexcept
    {
        _comm_thread_running = false;
        if (_comm_thread.joinable())
        {
            _comm_thread.join();
        }
    }

    void ST3215SystemHardware::stop_all_servos(const bool disable_torque) noexcept
    {
        if (!_serial_port.is_open())
        {
            return;
        }

        std::lock_guard<std::mutex> lock(_serial_mutex);
        const uint8_t goal_speed_register = static_cast<uint8_t>(ServoSramRegister::GOAL_SPEED_L);
        const uint8_t torque_register = static_cast<uint8_t>(ServoSramRegister::TORQUE_ENABLE);
        for (uint8_t servo_id : _servo_ids)
        {
            if (!send_servo_command(servo_id, ServoCommand::WRITE, std::array<uint8_t, 3>{goal_speed_register, 0, 0}))
            {
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Failed to stop servo %d", servo_id);
            }
            if (disable_torque &&
                !send_servo_command(servo_id, ServoCommand::WRITE, std::array<uint8_t, 2>{torque_register, _TORQUE_DISABLE_VALUE}))
            {
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Failed to disable torque for servo %d", servo_id);
            }
        }
    }

    hardware_interface::CallbackReturn ST3215SystemHardware::on_cleanup(
        const rclcpp_lifecycle::State&)
    {
        stop_communication_thread();

        if (_serial_port.is_open())
        {
            stop_all_servos(true);
            boost::system::error_code ignored;
            _serial_port.close(ignored);
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ST3215SystemHardware::on_shutdown(
        const rclcpp_lifecycle::State& previous_state)
    {
        return on_cleanup(previous_state);
    }

    hardware_interface::CallbackReturn ST3215SystemHardware::on_error(
        const rclcpp_lifecycle::State&)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Hardware error - stopping all servos");
        std::fill(_command_speeds.begin(), _command_speeds.end(), 0.0);
        stop_communication_thread();
        stop_all_servos(true);
        boost::system::error_code ignored;
        _serial_port.close(ignored);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ST3215SystemHardware::on_activate(
        const rclcpp_lifecycle::State&)
    {
        // Reset command speeds to prevent motion on activation
        std::fill(_command_speeds.begin(), _command_speeds.end(), 0.0);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ST3215SystemHardware::on_deactivate(
        const rclcpp_lifecycle::State&)
    {
        // Stop all servos
        std::fill(_command_speeds.begin(), _command_speeds.end(), 0.0);
        auto result = write(rclcpp::Time(), rclcpp::Duration::from_seconds(0));
        return (result == hardware_interface::return_type::OK)
                   ? hardware_interface::CallbackReturn::SUCCESS
                   : hardware_interface::CallbackReturn::ERROR;
    }

    hardware_interface::return_type ST3215SystemHardware::read(
        const rclcpp::Time&, const rclcpp::Duration&)
    {
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Reading servo states");
        std::lock_guard<std::mutex> lock(_state_mutex);

        const auto now = std::chrono::steady_clock::now();
        for (size_t i = 0; i < _servo_ids.size(); ++i)
        {
            const auto& state = _servo_states[i];

            if (!state.last_update || now - *state.last_update > _SERVO_TIMEOUT)
            {
                RCLCPP_WARN_THROTTLE(rclcpp::get_logger(LOGGER_NAME),
                                     steady_log_clock(), 1000,  // Warn every 1 second
                                     "No response from servo %d for more than %ld seconds - reporting zero velocity",
                                     _servo_ids[i], static_cast<long>(_SERVO_TIMEOUT.count()));

                // Stale feedback: report standstill and hold the last known
                // position rather than integrating outdated data.
                _current_velocities[i] = 0.0;
                _temperatures[i] = _ROOM_TEMPERATURE_CELSIUS;
                continue;
            }

            // Mirror the command inversion in write() so diff_drive_controller
            // sees both sides turning forward when the rover drives forward.
            const double direction = protocol::apply_motor_direction(_servo_ids[i], 1.0, _inverted_servo_ids);
            _current_positions[i] = direction * state.position;
            _current_velocities[i] = direction * state.velocity;
            _temperatures[i] = state.temperature;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ST3215SystemHardware::write(
        const rclcpp::Time&, const rclcpp::Duration&)
    {
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Writing servo commands");
        std::lock_guard<std::mutex> lock(_serial_mutex);

        try
        {
            for (size_t i = 0; i < _servo_ids.size(); ++i)
            {
                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                             "Servo %d - Input command speed (rad/s): %f",
                             _servo_ids[i], _command_speeds[i]);

                double corrected_speed = protocol::apply_motor_direction(
                    _servo_ids[i], _command_speeds[i], _inverted_servo_ids);

                uint16_t servo_speed = protocol::encode_servo_velocity(corrected_speed, _servo_max_rpm);

                const uint8_t goal_speed_register = static_cast<uint8_t>(ServoSramRegister::GOAL_SPEED_L);
                const std::array<uint8_t, 3> vel_data{
                    goal_speed_register,
                    static_cast<uint8_t>(servo_speed & 0xFF),
                    static_cast<uint8_t>((servo_speed >> 8) & 0xFF)};

                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                             "Servo %d - Final velocity bytes: 0x%02X 0x%02X",
                             _servo_ids[i], vel_data[1], vel_data[2]);

                if (!send_servo_command(_servo_ids[i], ServoCommand::WRITE, std::span{vel_data}))
                {
                    RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                "Failed to send velocity command to servo %d", _servo_ids[i]);
                    return hardware_interface::return_type::ERROR;
                }
            }

            return hardware_interface::return_type::OK;
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                         "Error writing to servos: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
    }

    bool ST3215SystemHardware::send_servo_command(
        const uint8_t id, const ServoCommand command,
        const std::span<const uint8_t> data) noexcept
    {
        auto packet = protocol::build_packet(
            id, static_cast<protocol::Command>(static_cast<uint8_t>(command)), data);

        // Debug output - convert to hex string for readable output
        std::stringstream debug_ss;
        debug_ss << "Sending servo command - ID: 0x" << std::hex << static_cast<int>(id)
                 << " CMD: 0x" << static_cast<int>(static_cast<uint8_t>(command)) << " Packet: ";
        for (const auto& byte : packet)
        {
            debug_ss << "0x" << std::setw(2) << std::setfill('0')
                     << static_cast<int>(byte) << " ";
        }
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "%s", debug_ss.str().c_str());

        try
        {
            boost::asio::write(_serial_port, boost::asio::buffer(packet));
            return true;
        }
        catch (const boost::system::system_error& e)
        {
            RCLCPP_ERROR_THROTTLE(rclcpp::get_logger(LOGGER_NAME), steady_log_clock(), 1000,
                                  "Failed to send command: %s", e.what());
            return false;
        }
    }

    bool ST3215SystemHardware::process_response(const std::span<const uint8_t> response,
                                                const uint8_t expected_id) noexcept
    {
        // Log raw response bytes for debugging
        {
            std::stringstream debug_ss;
            debug_ss << "Raw response bytes: ";
            for (const auto& byte : response)
            {
                debug_ss << std::hex << std::setw(2) << std::setfill('0')
                         << static_cast<int>(byte) << " ";
            }
            RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "%s", debug_ss.str().c_str());
        }

        bool got_expected = false;
        for (const auto& [id, payload] : protocol::extract_packets(response))
        {
            // Find matching servo ID
            const auto it = std::find(_servo_ids.begin(), _servo_ids.end(), id);
            if (it == _servo_ids.end())
            {
                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                             "Received packet for unknown servo ID %d", id);
                continue;
            }

            const auto index = static_cast<size_t>(std::distance(_servo_ids.begin(), it));
            RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                         "Processing packet for servo ID %d (index %zu)", id, index);

            if (!payload.empty())
            {
                const uint8_t error_byte = payload[protocol::ERROR_BYTE_INDEX];

                // Check error flags if present
                if (error_byte != 0)
                {
                    if (error_byte & static_cast<uint8_t>(ServoErrorFlag::INPUT_VOLTAGE))
                        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                    "Servo %d: Input Voltage Error", id);
                    if (error_byte & static_cast<uint8_t>(ServoErrorFlag::ANGLE_LIMIT))
                        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                    "Servo %d: Angle Limit Error", id);
                    if (error_byte & static_cast<uint8_t>(ServoErrorFlag::OVERHEATING))
                        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                    "Servo %d: Overheating Error", id);
                    if (error_byte & static_cast<uint8_t>(ServoErrorFlag::RANGE))
                        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                    "Servo %d: Range Error", id);
                    if (error_byte & static_cast<uint8_t>(ServoErrorFlag::CHECKSUM))
                        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                    "Servo %d: Checksum Error", id);
                    if (error_byte & static_cast<uint8_t>(ServoErrorFlag::OVERLOAD))
                        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                    "Servo %d: Overload Error", id);
                    if (error_byte & static_cast<uint8_t>(ServoErrorFlag::INSTRUCTION))
                        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                    "Servo %d: Instruction Error", id);
                }
            }

            // Only a full status reply counts as feedback. Short packets (the
            // acknowledgements for WRITE commands) must not refresh the
            // timestamp, or a servo that only ACKs would look alive.
            const auto status = protocol::parse_status_payload(payload, _servo_max_rpm);
            if (!status)
            {
                continue;
            }

            std::lock_guard<std::mutex> state_lock(_state_mutex);
            auto& state = _servo_states[index];

            // Wheel-mode position wraps every revolution; integrate the
            // shortest signed tick delta into a continuous joint angle.
            if (state.last_raw_position)
            {
                state.position += protocol::ticks_to_radians(static_cast<int16_t>(
                    protocol::position_delta_ticks(*state.last_raw_position, status->raw_position)));
            }
            else
            {
                state.position = protocol::ticks_to_radians(static_cast<int16_t>(status->raw_position));
            }
            state.last_raw_position = status->raw_position;
            state.velocity = status->velocity_rad_s;
            state.temperature = status->temperature_c;
            state.last_update = std::chrono::steady_clock::now();

            RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                         "Servo %d state updated - Pos: %.2f rad, Vel: %.2f rad/s, Temp: %.1f°C",
                         id, state.position, state.velocity, state.temperature);

            if (id == expected_id)
            {
                got_expected = true;
            }
        }
        return got_expected;
    }

    bool ST3215SystemHardware::verify_command_interfaces(
        const hardware_interface::ComponentInfo& joint_info,
        const rclcpp::Logger& logger) const
    {
        if (joint_info.command_interfaces.size() != 1)
        {
            RCLCPP_ERROR(logger,
                         "Joint '%s' has %zu command interfaces. Expected 1 (velocity)",
                         joint_info.name.c_str(), joint_info.command_interfaces.size());
            return false;
        }

        // Verify velocity interface
        if (joint_info.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_ERROR(logger,
                         "Joint '%s' missing velocity command interface",
                         joint_info.name.c_str());
            return false;
        }

        return true;
    }

}  // namespace perseus_lite_hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    perseus_lite_hardware::ST3215SystemHardware,
    hardware_interface::SystemInterface)
