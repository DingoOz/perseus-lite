#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <vector>

#include "i2c_imu_driver/i2c_device.hpp"
#include "i2c_imu_driver/imu_data_types.hpp"
#include "i2c_imu_driver/imu_device_config.hpp"

namespace i2c_imu_driver
{

    /**
     * @brief ROS2 node for I2C IMU sensor
     *
     * This node provides a standalone ROS2 interface for I2C IMU sensors.
     * It publishes sensor_msgs/Imu messages with configurable update rate,
     * handles graceful failure when IMU is not available, and provides
     * parameter-based configuration.
     */
    class I2cImuNode : public rclcpp::Node
    {
    public:
        /**
         * @brief Construct a new I2cImuNode object
         *
         * @param options Node options for ROS2 configuration
         */
        explicit I2cImuNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        /**
         * @brief Destroy the I2cImuNode object
         */
        ~I2cImuNode();

    private:
        /**
         * @brief Initialize node parameters
         */
        void _initialize_parameters();

        /**
         * @brief Initialize ROS2 publishers
         */
        void _initialize_publishers();

        /**
         * @brief Initialize I2C device
         *
         * @return true if initialization succeeded, false otherwise
         */
        bool _initialize_device();

        /**
         * @brief Timer callback for reading and publishing sensor data
         */
        void _timer_callback();

        /**
         * @brief Read raw sensor data from I2C device
         *
         * @return Raw sensor data, or std::nullopt if the accelerometer or
         *         gyroscope block could not be read (a partial reading must
         *         never be published as a valid sample)
         */
        std::optional<ImuData> _read_sensor_data();

        /**
         * @brief Record a failed read and, once the failure threshold is
         *        reached, tear down and re-initialise the device
         */
        void _handle_read_failure();

        /**
         * @brief Attempt to re-open and re-configure the device (rate limited)
         */
        void _attempt_reinitialize();

        /**
         * @brief Parse a covariance parameter
         *
         * Accepts either a 3-element diagonal or a full row-major 3x3 (9-element)
         * matrix. Anything else is rejected with an error log and @p fallback is
         * returned.
         */
        std::array<double, 9> _parse_covariance(const std::string& name,
                                                const std::vector<double>& values,
                                                const std::array<double, 9>& fallback);

        /**
         * @brief Convert IMU data to ROS message
         *
         * @param imu_data Calibrated IMU data
         * @return sensor_msgs::msg::Imu ROS IMU message
         */
        sensor_msgs::msg::Imu _convert_to_ros_message(const ImuData& imu_data);

        /**
         * @brief Initialize IMU sensor configuration
         *
         * @return true if initialization succeeded, false otherwise
         */
        bool _initialize_sensor();

        // Node parameters
        std::string _i2c_bus_path;
        uint8_t _device_address{0x6A};
        double _update_rate{100.0};
        std::string _frame_id;
        bool _required{false};
        int _retry_count{3};
        int _max_consecutive_failures{10};
        double _reinit_interval{1.0};

        // Device configuration
        std::string _device_type;
        const ImuDeviceConfig* _device_config{nullptr};

        // IMU calibration parameters
        double _accel_scale_x{1.0};
        double _accel_scale_y{1.0};
        double _accel_scale_z{1.0};
        double _accel_offset_x{0.0};
        double _accel_offset_y{0.0};
        double _accel_offset_z{0.0};

        double _gyro_scale_x{1.0};
        double _gyro_scale_y{1.0};
        double _gyro_scale_z{1.0};
        double _gyro_offset_x{0.0};
        double _gyro_offset_y{0.0};
        double _gyro_offset_z{0.0};

        // Covariance matrices (orientation not estimated by IMU). Value-initialised
        // to sane diagonal defaults so a rejected parameter never leaves them
        // indeterminate.
        static constexpr std::array<double, 9> DEFAULT_ANGULAR_VELOCITY_COVARIANCE{
            0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
        static constexpr std::array<double, 9> DEFAULT_LINEAR_ACCELERATION_COVARIANCE{
            0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1};
        std::array<double, 9> _angular_velocity_covariance{DEFAULT_ANGULAR_VELOCITY_COVARIANCE};
        std::array<double, 9> _linear_acceleration_covariance{DEFAULT_LINEAR_ACCELERATION_COVARIANCE};

        // ROS2 components
        std::unique_ptr<I2cDevice> _i2c_device;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_publisher;
        rclcpp::TimerBase::SharedPtr _timer;

        // Status tracking
        std::atomic<bool> _device_initialized{false};
        size_t _sequence_number{0};
        int _consecutive_failures{0};
        std::chrono::steady_clock::time_point _last_reinit_attempt{};
    };

}  // namespace i2c_imu_driver