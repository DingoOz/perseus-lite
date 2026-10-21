#pragma once

#include <atomic>
#include <boost/asio.hpp>
#include <chrono>
#include <hardware_interface/hardware_component_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <iomanip>
#include <mutex>
#include <numbers>
#include <optional>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <span>
#include <sstream>
#include <string_view>
#include <thread>
#include <vector>

#include "st3215_protocol.hpp"

namespace perseus_lite_hardware
{
    // EPROM registers
    enum class ServoEpromRegister : uint8_t
    {
        MODEL_L = 3,
        MODEL_H = 4,
        ID = 5,
        BAUD_RATE = 6,
        MIN_ANGLE_LIMIT_L = 9,
        MIN_ANGLE_LIMIT_H = 10,
        MAX_ANGLE_LIMIT_L = 11,
        MAX_ANGLE_LIMIT_H = 12,
        CW_DEAD = 26,
        CCW_DEAD = 27,
        OFS_L = 31,
        OFS_H = 32,
        MODE = 33
    };

    // SRAM registers
    enum class ServoSramRegister : uint8_t
    {
        TORQUE_ENABLE = 40,
        ACC = 41,
        GOAL_POSITION_L = 42,
        GOAL_POSITION_H = 43,
        GOAL_TIME_L = 44,
        GOAL_TIME_H = 45,
        GOAL_SPEED_L = 46,
        GOAL_SPEED_H = 47,
        TORQUE_LIMIT_L = 48,
        TORQUE_LIMIT_H = 49,
        LOCK = 55,
        PRESENT_POSITION_L = 56,
        PRESENT_POSITION_H = 57,
        PRESENT_SPEED_L = 58,
        PRESENT_SPEED_H = 59,
        PRESENT_LOAD_L = 60,
        PRESENT_LOAD_H = 61,
        PRESENT_VOLTAGE = 62,
        PRESENT_TEMPERATURE = 63,
        MOVING = 66,
        PRESENT_CURRENT_L = 69,
        PRESENT_CURRENT_H = 70
    };

    // Command codes
    enum class ServoCommand : uint8_t
    {
        READ = 0x02,
        WRITE = 0x03
    };

    // Servo error flags
    enum class ServoErrorFlag : uint8_t
    {
        INPUT_VOLTAGE = 0x01,
        ANGLE_LIMIT = 0x02,
        OVERHEATING = 0x04,
        RANGE = 0x08,
        CHECKSUM = 0x10,
        OVERLOAD = 0x20,
        INSTRUCTION = 0x40
    };

    class ST3215SystemHardware final : public hardware_interface::SystemInterface
    {
    public:
        ST3215SystemHardware() = default;
        ~ST3215SystemHardware() override;

        // Prevent copying and moving
        ST3215SystemHardware(const ST3215SystemHardware&) = delete;
        ST3215SystemHardware& operator=(const ST3215SystemHardware&) = delete;
        ST3215SystemHardware(ST3215SystemHardware&&) = delete;
        ST3215SystemHardware& operator=(ST3215SystemHardware&&) = delete;

        // ROS 2 Control Functions
        [[nodiscard]] hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams& params) override;

        [[nodiscard]] std::vector<hardware_interface::StateInterface>
        export_state_interfaces() override;

        [[nodiscard]] std::vector<hardware_interface::CommandInterface>
        export_command_interfaces() override;

        [[nodiscard]] hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State& previous_state) override;

        [[nodiscard]] hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State& previous_state) override;

        [[nodiscard]] hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& previous_state) override;

        [[nodiscard]] hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State& previous_state) override;

        /**
         * @brief Called by ros2_control when read()/write() fail or a transition errors
         * @details The servos have no communication watchdog and keep running at
         *          their last goal speed, so this stops them before giving up the bus.
         */
        [[nodiscard]] hardware_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State& previous_state) override;

        [[nodiscard]] hardware_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State& previous_state) override;

        [[nodiscard]] hardware_interface::return_type read(
            const rclcpp::Time& time, const rclcpp::Duration& period) override;

        [[nodiscard]] hardware_interface::return_type write(
            const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        // Constants for ST3215 servo specifications
        static constexpr size_t _BUFFER_SIZE = 256;              // Buffer size for serial communication
        static constexpr uint8_t _WHEEL_MODE_VALUE = 1;          // Value for wheel mode setting
        static constexpr uint8_t _TORQUE_ENABLE_VALUE = 1;       // Value to enable torque
        static constexpr uint8_t _TORQUE_DISABLE_VALUE = 0;      // Value to disable torque
        static constexpr uint8_t _DEFAULT_ACCELERATION = 0;      // 0 = no ramp (tuned on the robot)
        static constexpr uint8_t _PRESENT_POSITION_REG = 0x38;   // Present position register
        static constexpr size_t _ROOM_TEMPERATURE_CELSIUS = 25;  // Default room temperature
        static constexpr int _STARTUP_PROBE_ATTEMPTS = 3;        // Status reads per servo at configure

        // Communication timing constants
        static constexpr auto _SERVO_TIMEOUT = std::chrono::seconds(1);
        static constexpr auto _COMMAND_DELAY = std::chrono::milliseconds(10);
        static constexpr auto _COMMUNICATION_CYCLE_DELAY = std::chrono::milliseconds(20);
        // A 14-byte status reply takes ~0.15 ms at 1 Mbps; the deadline only
        // matters for a missing servo, and bounds how long write() can be
        // blocked behind a status transaction.
        static constexpr auto _RESPONSE_TIMEOUT = std::chrono::milliseconds(20);

        // Communication thread control
        std::atomic<bool> _comm_thread_running{false};
        std::thread _comm_thread;

        // Thread-safe queues for commands and responses
        mutable std::mutex _state_mutex;  // Protects servo state updates

        // Servo state tracking
        // Feedback is stamped and checked with std::chrono::steady_clock only:
        // mixing rclcpp clock types (ROS vs steady) throws on subtraction.
        struct ServoState
        {
            double position{0.0};  // unwrapped, continuous, in the servo's own direction
            double velocity{0.0};
            double temperature{_ROOM_TEMPERATURE_CELSIUS};
            std::optional<uint16_t> last_raw_position;
            std::optional<std::chrono::steady_clock::time_point> last_update;
        };
        std::vector<ServoState> _servo_states;

        // Thread management methods
        /**
         * @brief Main communication thread function that continuously reads servo states
         * @details Runs in a separate thread to handle asynchronous communication with servos.
         *          Reads position, velocity, and temperature data from all configured servos.
         */
        void communication_thread() noexcept;

        /**
         * @brief Sends a command packet to a specific servo
         * @param id The servo ID to send the command to
         * @param cmd The command type (READ or WRITE)
         * @param data The data payload for the command
         * @return true if the command was sent successfully, false otherwise
         * @details Constructs and transmits a properly formatted ST3215 protocol packet
         */
        bool send_servo_command(uint8_t id, ServoCommand cmd, std::span<const uint8_t> data) noexcept;
        /**
         * @brief Requests and waits for one servo's status (caller holds _serial_mutex)
         * @return true if a valid status packet from that servo was received and applied
         * @details Discards stale input (e.g. the acknowledgements servos send for
         *          WRITE commands) before the request so that only a real status
         *          reply can refresh the servo's feedback timestamp.
         */
        bool request_status(size_t index) noexcept;
        /**
         * @brief Applies a status packet from a servo to its tracked state
         * @return true if the packet was a status reply from the expected servo
         */
        bool process_response(std::span<const uint8_t> response, uint8_t expected_id) noexcept;
        /**
         * @brief Commands zero speed to every servo, optionally disabling torque
         * @details Best effort: errors are logged, not thrown. Caller must not hold
         *          _serial_mutex.
         */
        void stop_all_servos(bool disable_torque) noexcept;
        void stop_communication_thread() noexcept;
        void discard_pending_input() noexcept;
        /**
         * @brief Reads available bytes from the serial port, giving up after a deadline
         * @param buffer Destination buffer for the read
         * @param timeout How long to wait for any bytes to arrive
         * @param error Set to the read error, or operation_aborted on timeout
         * @return The number of bytes read (0 on timeout or error)
         * @details Runs an async read against a deadline timer on _io_context so a
         *          wedged servo or unplugged adapter cannot block the thread forever.
         */
        [[nodiscard]] size_t read_with_timeout(std::span<uint8_t> buffer,
                                               std::chrono::milliseconds timeout,
                                               boost::system::error_code& error);
        /**
         * @brief Verifies that required command interfaces are available for a joint
         * @param joint_info Information about the joint configuration
         * @param logger Logger instance for error reporting
         * @return true if all required interfaces are present, false otherwise
         * @details Checks that velocity command interface is configured for the joint
         */
        [[nodiscard]] bool verify_command_interfaces(
            const hardware_interface::ComponentInfo& joint_info,
            const rclcpp::Logger& logger) const;

        // Hardware state storage
        std::vector<double> _command_speeds;      // Only velocity commands
        std::vector<double> _current_positions;   // State feedback
        std::vector<double> _current_velocities;  // State feedback
        std::vector<double> _temperatures;        // State feedback
        std::vector<uint8_t> _servo_ids;
        // Servos mounted mirrored (left side): commands AND feedback are negated
        // so every joint reports positive = forward to diff_drive_controller.
        std::vector<uint8_t> _inverted_servo_ids;
        double _servo_max_rpm{protocol::DEFAULT_SERVO_MAX_RPM};
        uint8_t _acceleration{_DEFAULT_ACCELERATION};

        // Communication members
        // _serial_mutex serializes complete request/response transactions on the
        // bus; _io_context is only ever run from the thread holding it.
        mutable std::mutex _serial_mutex;
        boost::asio::io_context _io_context;
        boost::asio::serial_port _serial_port{_io_context};
    };

}  // namespace perseus_lite_hardware