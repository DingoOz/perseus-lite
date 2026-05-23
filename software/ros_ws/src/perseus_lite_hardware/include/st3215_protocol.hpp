#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <numeric>
#include <span>
#include <vector>

namespace perseus_lite_hardware::protocol
{
    static constexpr double RADIANS_PER_REVOLUTION = 2.0 * std::numbers::pi;
    static constexpr double SECONDS_PER_MINUTE = 60.0;
    static constexpr double RPM_TO_RAD_S = RADIANS_PER_REVOLUTION / SECONDS_PER_MINUTE;
    static constexpr double RAD_S_TO_RPM = SECONDS_PER_MINUTE / RADIANS_PER_REVOLUTION;
    static constexpr uint16_t ENCODER_TICKS_PER_REVOLUTION = 4096;
    static constexpr double RPM_SCALE_FACTOR = 7.5;
    static constexpr int16_t MAX_VELOCITY_RPM = 1000;
    static constexpr int16_t MIN_VELOCITY_RPM = -1000;
    static constexpr uint16_t SIGN_BIT_MASK = 1 << 15;

    static constexpr uint8_t PACKET_HEADER_BYTE = 0xFF;
    static constexpr size_t PACKET_HEADER_SIZE = 2;
    static constexpr size_t PACKET_ID_INDEX = 2;

    enum class Command : uint8_t
    {
        READ = 0x02,
        WRITE = 0x03
    };

    inline uint8_t calculate_checksum(std::span<const uint8_t> body)
    {
        return ~std::accumulate(body.begin(), body.end(), uint8_t{0});
    }

    inline std::vector<uint8_t> build_packet(uint8_t id, Command cmd, std::span<const uint8_t> data)
    {
        std::vector<uint8_t> packet;
        packet.reserve(data.size() + 6);

        packet.push_back(PACKET_HEADER_BYTE);
        packet.push_back(PACKET_HEADER_BYTE);
        packet.push_back(id);
        packet.push_back(static_cast<uint8_t>(data.size() + 2));
        packet.push_back(static_cast<uint8_t>(cmd));
        packet.insert(packet.end(), data.begin(), data.end());

        const uint8_t checksum = calculate_checksum(
            std::span{packet.data() + PACKET_ID_INDEX, packet.size() - PACKET_ID_INDEX});
        packet.push_back(checksum);

        return packet;
    }

    inline int16_t parse_signed_value(uint16_t raw)
    {
        if (raw & SIGN_BIT_MASK)
        {
            return -static_cast<int16_t>(raw & ~SIGN_BIT_MASK);
        }
        return static_cast<int16_t>(raw);
    }

    inline double ticks_to_radians(int16_t ticks)
    {
        return ticks * (RADIANS_PER_REVOLUTION / ENCODER_TICKS_PER_REVOLUTION);
    }

    inline double raw_velocity_to_rad_s(int16_t raw_velocity)
    {
        const double rpm = raw_velocity * (RPM_SCALE_FACTOR / MAX_VELOCITY_RPM);
        return rpm * RPM_TO_RAD_S;
    }

    inline uint16_t encode_servo_velocity(double rad_s)
    {
        const double rpm = rad_s * RAD_S_TO_RPM;
        double scaled = rpm * (MAX_VELOCITY_RPM / RPM_SCALE_FACTOR);
        double clamped = std::clamp(scaled, static_cast<double>(MIN_VELOCITY_RPM),
                                    static_cast<double>(MAX_VELOCITY_RPM));

        auto servo_speed = static_cast<int16_t>(clamped);
        if (servo_speed < 0)
        {
            servo_speed = -servo_speed;
            servo_speed |= static_cast<int16_t>(SIGN_BIT_MASK);
        }
        return static_cast<uint16_t>(servo_speed);
    }

    inline double apply_motor_direction(uint8_t servo_id, double velocity)
    {
        // Servo IDs 2 and 3 are left-side motors that need direction inversion
        if (servo_id == 2 || servo_id == 3)
        {
            return -velocity;
        }
        return velocity;
    }

}  // namespace perseus_lite_hardware::protocol
