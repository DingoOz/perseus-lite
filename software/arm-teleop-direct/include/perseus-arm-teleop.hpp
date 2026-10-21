#pragma once

// Forward declare WINDOW type to avoid ncurses/boost conflicts
struct _win_st;
typedef struct _win_st WINDOW;

#include <fcntl.h>
#include <termios.h>

#include <boost/asio.hpp>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "st3215-protocol.hpp"

/// Thrown when a servo does not answer within the deadline.
class SerialTimeoutError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

class ST3215ServoReader
{
public:
    /**
     * @brief Constructs a new ST3215ServoReader
     * @param port Serial port path (e.g., "/dev/ttyACM0")
     * @param baud_rate Baud rate for serial communication
     * @param acceleration Acceleration value for servo movement (0-255)
     */
    ST3215ServoReader(const std::string& port, unsigned int baud_rate, uint8_t acceleration = 30);

    /**
     * @brief Destructor ensures serial port is properly closed
     */
    ~ST3215ServoReader();

    ST3215ServoReader(const ST3215ServoReader&) = delete;
    ST3215ServoReader& operator=(const ST3215ServoReader&) = delete;

    /**
     * @brief Reads the current position of a specified servo
     * @param servo_id ID of the servo to read from
     * @return Current position value (0-4095)
     * @throws std::runtime_error if communication fails after retries
     */
    uint16_t read_position(uint8_t servo_id);

    /**
     * @brief Writes a position value to a specified servo
     * @param servo_id ID of the servo to write to
     * @param position Position value to write (0-4095)
     * @throws std::runtime_error if the write fails or the servo reports an error
     */
    void write_position(uint8_t servo_id, uint16_t position);

    /**
     * @brief Writes to a control register of the servo. The servo's status
     *        reply (if any) is consumed but not checked.
     * @param servo_id ID of the servo to write to
     * @param address Register address
     * @param value Value to write
     * @throws std::runtime_error if the write itself fails
     */
    void write_control_register(uint8_t servo_id, uint8_t address, uint8_t value);

    /**
     * @brief Reads the current load/torque value from a specified servo
     * @param servo_id ID of the servo to read from
     * @return Current load value (-1000 to +1000, sign = direction)
     * @throws std::runtime_error if communication fails after retries
     */
    int16_t read_load(uint8_t servo_id);

    /**
     * @brief Gets a reference to the serial port for low-level operations
     * @return Reference to the serial port object
     */
    boost::asio::serial_port& get_serial_port() { return _serial_port; }

private:
    /**
     * @brief Creates a write command packet according to ST3215 protocol
     * @param id Servo ID
     * @param address Memory address to write to
     * @param data Data bytes to write
     * @return Vector containing the complete command packet
     */
    std::vector<uint8_t> _create_write_command(uint8_t id, uint8_t address, const std::vector<uint8_t>& data);

    /**
     * @brief Creates a read command packet according to ST3215 protocol
     * @param id Servo ID
     * @param address Memory address to read from
     * @param size Number of bytes to read
     * @return Vector containing the complete command packet
     */
    std::vector<uint8_t> _create_read_command(uint8_t id, uint8_t address, uint8_t size);

    /**
     * @brief One register-read transaction (flush, request, validated reply)
     * @return The register bytes
     * @throws SerialTimeoutError, perseus::servo::st3215::ProtocolError or
     *         std::runtime_error (servo error bits / I/O failure)
     */
    std::vector<uint8_t> _read_register_once(uint8_t servo_id, uint8_t address, uint8_t size,
                                             std::chrono::milliseconds timeout);

    /// _read_register_once() with communication::MAX_RETRIES attempts.
    std::vector<uint8_t> _read_register(uint8_t servo_id, uint8_t address, uint8_t size);

    /// Writes a complete command packet (throws on I/O error / short write).
    void _send(const std::vector<uint8_t>& command);

    /**
     * @brief Reads exactly @p size bytes, or throws SerialTimeoutError once
     *        @p deadline passes. Uses async_read + io_context::run_until so a
     *        missing servo can never block the caller forever.
     */
    void _read_exact(uint8_t* data, std::size_t size, std::chrono::steady_clock::time_point deadline);

    /**
     * @brief Reads one status packet (resynchronising on the 0xFF 0xFF header)
     *        and verifies its checksum and servo ID.
     */
    perseus::servo::st3215::StatusPacket _read_status(uint8_t servo_id, std::chrono::milliseconds timeout);

    /// Discards any unread input and unsent output on the port.
    void _flush();

    boost::asio::io_context _io_context;
    boost::asio::serial_port _serial_port;
};
