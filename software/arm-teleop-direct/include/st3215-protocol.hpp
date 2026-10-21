#pragma once

// Pure (I/O-free) helpers for the Feetech ST3215 serial protocol: checksum
// computation/verification, status-packet validation and register decoding.
// Kept header-only and free of Boost/ncurses so they can be unit tested.

#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "servo-constants.hpp"

namespace perseus::servo::st3215
{
    /**
     * @brief Feetech checksum: bitwise NOT of the byte sum, truncated to 8 bits.
     * @param bytes Every byte after the 0xFF 0xFF header up to (not including)
     *              the checksum itself, ie ID, LENGTH, then INSTRUCTION/ERROR
     *              and the parameters.
     */
    inline uint8_t checksum(std::span<const uint8_t> bytes)
    {
        unsigned int sum = 0;
        for (const uint8_t b : bytes)
        {
            sum += b;
        }
        return static_cast<uint8_t>(~sum & 0xFF);
    }

    /// Thrown when a reply is structurally invalid or fails its checksum.
    class ProtocolError : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    /// A decoded status (reply) packet.
    struct StatusPacket
    {
        uint8_t id = 0;
        uint8_t error = 0;
        std::vector<uint8_t> params;
    };

    /**
     * @brief Validates and decodes a status packet body.
     *
     * Wire format: FF FF ID LEN ERR P1..Pn CHK, where LEN = n + 2 and
     * CHK = ~(ID + LEN + ERR + P1 + ... + Pn) & 0xFF.
     *
     * @param id   The ID byte that followed the header.
     * @param body The LEN bytes that followed the length byte (ERR, params, CHK).
     * @throws ProtocolError on a bad length or checksum mismatch.
     */
    inline StatusPacket parse_status_body(uint8_t id, std::span<const uint8_t> body)
    {
        if (body.size() < protocol::MIN_STATUS_LENGTH)
        {
            throw ProtocolError("Invalid status packet length");
        }
        if (body.size() > 0xFF)
        {
            throw ProtocolError("Status packet too long");
        }

        std::vector<uint8_t> summed;
        summed.reserve(body.size() + 1);
        summed.push_back(id);
        summed.push_back(static_cast<uint8_t>(body.size()));
        summed.insert(summed.end(), body.begin(), body.end() - 1);
        if (checksum(summed) != body.back())
        {
            throw ProtocolError("Checksum mismatch in status packet");
        }

        StatusPacket packet;
        packet.id = id;
        packet.error = body.front();
        packet.params.assign(body.begin() + 1, body.end() - 1);
        return packet;
    }

    /// Little-endian 16-bit word from two bytes (low byte first).
    inline uint16_t word_le(uint8_t low, uint8_t high)
    {
        return static_cast<uint16_t>(low | (static_cast<uint16_t>(high) << 8));
    }

    /**
     * @brief Decodes a Feetech sign-magnitude register value.
     *
     * Feetech status registers do NOT use two's complement: the bit at
     * @p sign_bit is the direction and the bits below it the magnitude.
     */
    inline int32_t decode_sign_magnitude(uint16_t raw, unsigned int sign_bit)
    {
        const uint16_t magnitude_mask = static_cast<uint16_t>((1U << sign_bit) - 1U);
        const int32_t magnitude = raw & magnitude_mask;
        return (raw & (1U << sign_bit)) ? -magnitude : magnitude;
    }

    /// Present load (0x3C): bit 10 = direction, bits 0-9 = magnitude (0-1000).
    inline int16_t decode_load(uint16_t raw)
    {
        return static_cast<int16_t>(decode_sign_magnitude(raw, encoding::LOAD_SIGN_BIT));
    }

    /// Present speed (0x3A): bit 15 = direction, bits 0-14 = magnitude.
    inline int16_t decode_speed(uint16_t raw)
    {
        return static_cast<int16_t>(decode_sign_magnitude(raw, encoding::SPEED_SIGN_BIT));
    }

    /**
     * @brief Present position (0x38): bit 15 = direction, bits 0-14 = magnitude.
     *
     * In the arm's position mode the value is always 0-4095; a negative value
     * (multi-turn/step modes only) is clamped to 0 and anything above the
     * 12-bit range to MAX_POSITION so callers always get a valid position.
     */
    inline uint16_t decode_position(uint16_t raw)
    {
        const int32_t value = decode_sign_magnitude(raw, encoding::POSITION_SIGN_BIT);
        if (value < limits::MIN_POSITION)
        {
            return limits::MIN_POSITION;
        }
        if (value > limits::MAX_POSITION)
        {
            return limits::MAX_POSITION;
        }
        return static_cast<uint16_t>(value);
    }

    /// Human-readable list of the error bits set in a status packet.
    inline std::string format_error(uint8_t error)
    {
        std::string text = "Servo errors:";
        if (error & error_bits::INPUT_VOLTAGE)
            text += " Input Voltage";
        if (error & error_bits::ANGLE_LIMIT)
            text += " Angle Limit";
        if (error & error_bits::OVERHEATING)
            text += " Overheating";
        if (error & error_bits::RANGE)
            text += " Range";
        if (error & error_bits::CHECKSUM)
            text += " Checksum";
        if (error & error_bits::OVERLOAD)
            text += " Overload";
        if (error & error_bits::INSTRUCTION)
            text += " Instruction";
        return text;
    }
}  // namespace perseus::servo::st3215
