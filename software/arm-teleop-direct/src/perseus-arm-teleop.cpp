// For controlling one pair of SO-100 arms

#include "perseus-arm-teleop.hpp"

#include <fcntl.h>
#include <termios.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "servo-constants.hpp"
#include "st3215-protocol.hpp"

using namespace perseus::servo;

namespace
{
    // cfsetspeed() only portably accepts B* constants (glibc's numeric-value
    // fallback does not cover every rate, eg 1000000), so map explicitly.
    speed_t to_speed_constant(unsigned int baud_rate)
    {
        switch (baud_rate)
        {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 500000:
            return B500000;
        case 576000:
            return B576000;
        case 921600:
            return B921600;
        case 1000000:
            return B1000000;
        case 1152000:
            return B1152000;
        case 1500000:
            return B1500000;
        case 2000000:
            return B2000000;
        default:
            throw std::runtime_error("Unsupported baud rate " + std::to_string(baud_rate));
        }
    }
}  // namespace

ST3215ServoReader::ST3215ServoReader(const std::string& port, unsigned int baud_rate, uint8_t acceleration)
    : _io_context(),
      _serial_port(_io_context)
{
    boost::system::error_code ec;
    _serial_port.open(port, ec);
    if (ec)
    {
        throw std::runtime_error("Failed to open serial port " + port + ": " + ec.message());
    }

    // Get the native handle for low-level configuration
    const int fd = _serial_port.native_handle();

    // Configure ACM port settings
    struct termios to;
    if (tcgetattr(fd, &to) != 0)
    {
        throw std::runtime_error("Failed to get port attributes");
    }

    // Input flags - disable break processing
    to.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

    // Output flags - disable post processing
    to.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

    // No line processing
    to.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

    // Clean character processing
    to.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB);
    to.c_cflag |= CS8 | CLOCAL | CREAD;

    // Set speed
    if (cfsetspeed(&to, to_speed_constant(baud_rate)) != 0)
    {
        throw std::runtime_error("Failed to set baud rate " + std::to_string(baud_rate));
    }

    // Reads never rely on VMIN/VTIME: every read is an async_read bounded by
    // a deadline on _io_context (see _read_exact), so a missing servo cannot
    // block forever.
    to.c_cc[VMIN] = 1;
    to.c_cc[VTIME] = 0;

    // Apply settings
    if (tcsetattr(fd, TCSANOW, &to) != 0)
    {
        throw std::runtime_error("Failed to apply port attributes");
    }

    _flush();

    // We still set the boost::asio options for consistency
    try
    {
        _serial_port.set_option(boost::asio::serial_port::baud_rate(baud_rate));
        _serial_port.set_option(boost::asio::serial_port::character_size(8));
        _serial_port.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        _serial_port.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        _serial_port.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    }
    catch (const boost::system::system_error& e)
    {
        throw std::runtime_error(std::string("Failed to configure serial port: ") + e.what());
    }

    // Initial delay to let port settle - ACM devices often need more time
    std::this_thread::sleep_for(timing::PORT_SETTLE_TIME);

    // Set acceleration for all servos
    for (uint8_t id = arm::FIRST_SERVO_ID; id < arm::FIRST_SERVO_ID + arm::SERVO_COUNT; ++id)
    {
        try
        {
            write_control_register(id, register_addr::ACCELERATION, acceleration);
            std::this_thread::sleep_for(timing::MIN_RESPONSE_TIME);  // Wait between writes
        }
        catch (const std::exception& e)
        {
            std::cerr << "Warning: Failed to set acceleration for servo " << static_cast<int>(id)
                      << ": " << e.what() << std::endl;
        }
    }
}

ST3215ServoReader::~ST3215ServoReader()
{
    if (_serial_port.is_open())
    {
        _flush();
        boost::system::error_code ignored;
        _serial_port.close(ignored);
    }
}

void ST3215ServoReader::_flush()
{
    ::tcflush(static_cast<int>(_serial_port.native_handle()), TCIOFLUSH);
}

void ST3215ServoReader::_send(const std::vector<uint8_t>& command)
{
    boost::system::error_code write_ec;
    const std::size_t written = boost::asio::write(_serial_port, boost::asio::buffer(command), write_ec);
    if (write_ec)
    {
        throw std::runtime_error(std::string("Write error: ") + write_ec.message());
    }
    if (written != command.size())
    {
        throw std::runtime_error("Failed to write complete command");
    }
}

void ST3215ServoReader::_read_exact(uint8_t* data, std::size_t size,
                                    std::chrono::steady_clock::time_point deadline)
{
    if (size == 0)
    {
        return;
    }
    if (std::chrono::steady_clock::now() >= deadline)
    {
        throw SerialTimeoutError("Timeout waiting for servo response");
    }

    bool done = false;
    boost::system::error_code result_ec;
    std::size_t transferred = 0;
    boost::asio::async_read(_serial_port, boost::asio::buffer(data, size),
                            [&done, &result_ec, &transferred](const boost::system::error_code& ec, std::size_t n)
                            {
                                done = true;
                                result_ec = ec;
                                transferred = n;
                            });

    _io_context.restart();
    _io_context.run_until(deadline);

    if (!done)
    {
        // Deadline hit: cancel the read and run the (now aborted) handler to
        // completion so nothing still references this stack frame.
        boost::system::error_code ignored;
        _serial_port.cancel(ignored);
        _io_context.restart();
        _io_context.run();
        if (!result_ec && transferred == size)
        {
            return;  // Completed in the window between the deadline and cancel()
        }
        throw SerialTimeoutError("Timeout waiting for servo response");
    }
    if (result_ec)
    {
        throw std::runtime_error(std::string("Serial read error: ") + result_ec.message());
    }
}

st3215::StatusPacket ST3215ServoReader::_read_status(uint8_t servo_id, std::chrono::milliseconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;

    // Resynchronise on the 0xFF 0xFF header: skip junk until two header bytes
    // are followed by a non-0xFF byte (the ID; 0xFF is never a valid ID).
    uint8_t byte = 0;
    std::size_t header_bytes = 0;
    std::size_t skipped = 0;
    while (true)
    {
        _read_exact(&byte, 1, deadline);
        if (byte == protocol::HEADER1)
        {
            ++header_bytes;
            continue;
        }
        if (header_bytes >= 2)
        {
            break;  // `byte` is the ID
        }
        header_bytes = 0;
        if (++skipped > protocol::MAX_SYNC_BYTES)
        {
            throw st3215::ProtocolError("No status packet header found");
        }
    }
    const uint8_t id = byte;

    uint8_t length = 0;
    _read_exact(&length, 1, deadline);
    if (length < protocol::MIN_STATUS_LENGTH)
    {
        throw st3215::ProtocolError("Invalid length");
    }

    std::vector<uint8_t> body(length);
    _read_exact(body.data(), body.size(), deadline);

    st3215::StatusPacket packet = st3215::parse_status_body(id, body);
    if (packet.id != servo_id)
    {
        throw st3215::ProtocolError("Mismatched servo ID");
    }
    return packet;
}

std::vector<uint8_t> ST3215ServoReader::_read_register_once(uint8_t servo_id, uint8_t address, uint8_t size,
                                                            std::chrono::milliseconds timeout)
{
    // Clear any stale data (eg a late reply to an earlier timed-out request)
    _flush();
    std::this_thread::sleep_for(timing::COMMAND_INTERVAL);

    _send(_create_read_command(servo_id, address, size));

    const st3215::StatusPacket packet = _read_status(servo_id, timeout);
    if (packet.error != 0x00)
    {
        throw std::runtime_error(st3215::format_error(packet.error));
    }
    if (packet.params.size() != size)
    {
        throw st3215::ProtocolError("Unexpected reply size");
    }
    return packet.params;
}

std::vector<uint8_t> ST3215ServoReader::_read_register(uint8_t servo_id, uint8_t address, uint8_t size)
{
    for (int retry = 0;; ++retry)
    {
        try
        {
            return _read_register_once(servo_id, address, size, timing::DEFAULT_TIMEOUT);
        }
        catch (const std::runtime_error&)
        {
            if (retry >= communication::MAX_RETRIES - 1)
            {
                throw;  // Re-throw if this was our last retry
            }
            _flush();
            std::this_thread::sleep_for(timing::RETRY_DELAY);
        }
    }
}

uint16_t ST3215ServoReader::read_position(uint8_t servo_id)
{
    const auto data = _read_register(servo_id, register_addr::PRESENT_POSITION, 2);
    return st3215::decode_position(st3215::word_le(data[0], data[1]));
}

int16_t ST3215ServoReader::read_load(uint8_t servo_id)
{
    const auto data = _read_register(servo_id, register_addr::PRESENT_LOAD, 2);
    return st3215::decode_load(st3215::word_le(data[0], data[1]));
}

std::vector<uint8_t> ST3215ServoReader::_create_read_command(uint8_t id, uint8_t address, uint8_t size)
{
    std::vector<uint8_t> command = {
        protocol::HEADER1,
        protocol::HEADER2,  // Header
        id,                 // Servo ID
        0x04,               // Length (always 4 for read command)
        instruction::READ,  // READ instruction
        address,            // Starting address
        size,               // Number of bytes to read
    };
    command.push_back(st3215::checksum(std::span<const uint8_t>(command).subspan(2)));
    return command;
}

std::vector<uint8_t> ST3215ServoReader::_create_write_command(uint8_t id, uint8_t address, const std::vector<uint8_t>& data)
{
    std::vector<uint8_t> command;
    command.reserve(data.size() + 7);  // Pre-allocate space

    // Header
    command.push_back(protocol::HEADER1);
    command.push_back(protocol::HEADER2);
    command.push_back(id);
    command.push_back(static_cast<uint8_t>(data.size() + 3));  // Length (data + instruction, address, checksum)
    command.push_back(instruction::WRITE);                     // WRITE instruction
    command.push_back(address);

    // Data
    command.insert(command.end(), data.begin(), data.end());

    command.push_back(st3215::checksum(std::span<const uint8_t>(command).subspan(2)));
    return command;
}

void ST3215ServoReader::write_position(uint8_t servo_id, uint16_t position)
{
    // Clamp position to valid range
    position = std::min(position, limits::MAX_POSITION);

    // Create data bytes (little-endian)
    const std::vector<uint8_t> data = {
        static_cast<uint8_t>(position & 0xFF),
        static_cast<uint8_t>((position >> 8) & 0xFF)};

    _flush();
    _send(_create_write_command(servo_id, register_addr::GOAL_POSITION, data));

    // Wait for minimum response time
    std::this_thread::sleep_for(timing::MIN_RESPONSE_TIME);

    // The status reply is optional (STATUS_RETURN_LEVEL) and a corrupted one
    // carries no trustworthy error byte, so only a valid reply reporting
    // error bits is treated as a failure.
    st3215::StatusPacket packet;
    try
    {
        packet = _read_status(servo_id, timing::STATUS_TIMEOUT);
    }
    catch (const SerialTimeoutError&)
    {
        return;
    }
    catch (const st3215::ProtocolError&)
    {
        return;
    }
    if (packet.error != 0x00)
    {
        throw std::runtime_error(st3215::format_error(packet.error));
    }
}

void ST3215ServoReader::write_control_register(uint8_t servo_id, uint8_t address, uint8_t value)
{
    _flush();
    _send(_create_write_command(servo_id, address, {value}));

    // Wait for minimum response time
    std::this_thread::sleep_for(timing::MIN_RESPONSE_TIME);

    // Consume the status reply, if any, so it can't be mistaken for the
    // answer to the next request. Its contents are deliberately not checked:
    // torque-off must still "succeed" on a servo that is reporting overload.
    try
    {
        (void)_read_status(servo_id, timing::STATUS_TIMEOUT);
    }
    catch (const SerialTimeoutError&)
    {
    }
    catch (const st3215::ProtocolError&)
    {
    }
}
