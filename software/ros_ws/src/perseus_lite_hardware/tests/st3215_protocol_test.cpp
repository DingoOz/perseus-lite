#include "st3215_protocol.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <vector>

namespace protocol = perseus_lite_hardware::protocol;

class ST3215ProtocolTest : public ::testing::Test
{
};

TEST_F(ST3215ProtocolTest, ChecksumSingleByte)
{
    const std::array<uint8_t, 1> data{0x00};
    EXPECT_EQ(protocol::calculate_checksum(data), 0xFF);
}

TEST_F(ST3215ProtocolTest, ChecksumMultipleBytes)
{
    const std::array<uint8_t, 3> data{0x01, 0x04, 0x02};
    uint8_t expected = ~(uint8_t)(0x01 + 0x04 + 0x02);
    EXPECT_EQ(protocol::calculate_checksum(data), expected);
}

TEST_F(ST3215ProtocolTest, ChecksumOverflow)
{
    const std::array<uint8_t, 2> data{0xFF, 0x01};
    uint8_t expected = ~(uint8_t)(0xFF + 0x01);
    EXPECT_EQ(protocol::calculate_checksum(data), expected);
}

TEST_F(ST3215ProtocolTest, BuildPacketHeader)
{
    const std::array<uint8_t, 2> data{0x38, 0x08};
    auto packet = protocol::build_packet(1, protocol::Command::READ, data);

    ASSERT_GE(packet.size(), 7u);
    EXPECT_EQ(packet[0], 0xFF);
    EXPECT_EQ(packet[1], 0xFF);
}

TEST_F(ST3215ProtocolTest, BuildPacketIdAndCommand)
{
    const std::array<uint8_t, 2> data{0x38, 0x08};
    auto packet = protocol::build_packet(3, protocol::Command::READ, data);

    EXPECT_EQ(packet[2], 3);
    EXPECT_EQ(packet[4], 0x02);
}

TEST_F(ST3215ProtocolTest, BuildPacketLength)
{
    const std::array<uint8_t, 2> data{0x38, 0x08};
    auto packet = protocol::build_packet(1, protocol::Command::READ, data);

    // Length = data size (2) + command (1) + checksum (1) = 4
    EXPECT_EQ(packet[3], 4);
}

TEST_F(ST3215ProtocolTest, BuildPacketData)
{
    const std::array<uint8_t, 2> data{0x38, 0x08};
    auto packet = protocol::build_packet(1, protocol::Command::READ, data);

    EXPECT_EQ(packet[5], 0x38);
    EXPECT_EQ(packet[6], 0x08);
}

TEST_F(ST3215ProtocolTest, BuildPacketChecksum)
{
    const std::array<uint8_t, 2> data{0x38, 0x08};
    auto packet = protocol::build_packet(1, protocol::Command::READ, data);

    // Checksum is ~(sum of bytes from ID to end of data)
    // Bytes: ID=1, Length=4, CMD=0x02, Data=0x38, 0x08
    uint8_t expected = ~(uint8_t)(1 + 4 + 0x02 + 0x38 + 0x08);
    EXPECT_EQ(packet.back(), expected);
}

TEST_F(ST3215ProtocolTest, BuildPacketTotalSize)
{
    const std::array<uint8_t, 3> data{0x2E, 0x00, 0x00};
    auto packet = protocol::build_packet(2, protocol::Command::WRITE, data);

    // Header(2) + ID(1) + Length(1) + CMD(1) + Data(3) + Checksum(1) = 9
    EXPECT_EQ(packet.size(), 9u);
}

TEST_F(ST3215ProtocolTest, BuildWritePacketCommand)
{
    const std::array<uint8_t, 2> data{0x21, 0x01};
    auto packet = protocol::build_packet(1, protocol::Command::WRITE, data);

    EXPECT_EQ(packet[4], 0x03);
}

TEST_F(ST3215ProtocolTest, ParseSignedValuePositive)
{
    EXPECT_EQ(protocol::parse_signed_value(100), 100);
    EXPECT_EQ(protocol::parse_signed_value(0), 0);
    EXPECT_EQ(protocol::parse_signed_value(2048), 2048);
}

TEST_F(ST3215ProtocolTest, ParseSignedValueNegative)
{
    // Bit 15 set: 0x8064 = sign bit | 100
    EXPECT_EQ(protocol::parse_signed_value(0x8064), -100);
    // 0x8001 = sign bit | 1
    EXPECT_EQ(protocol::parse_signed_value(0x8001), -1);
}

TEST_F(ST3215ProtocolTest, ParseSignedValueZeroWithSignBit)
{
    EXPECT_EQ(protocol::parse_signed_value(0x8000), 0);
}

TEST_F(ST3215ProtocolTest, TicksToRadiansZero)
{
    EXPECT_DOUBLE_EQ(protocol::ticks_to_radians(0), 0.0);
}

TEST_F(ST3215ProtocolTest, TicksToRadiansFullRevolution)
{
    EXPECT_NEAR(protocol::ticks_to_radians(4096), 2.0 * std::numbers::pi, 1e-10);
}

TEST_F(ST3215ProtocolTest, TicksToRadiansHalfRevolution)
{
    EXPECT_NEAR(protocol::ticks_to_radians(2048), std::numbers::pi, 1e-10);
}

TEST_F(ST3215ProtocolTest, TicksToRadiansNegative)
{
    EXPECT_NEAR(protocol::ticks_to_radians(-2048), -std::numbers::pi, 1e-10);
}

TEST_F(ST3215ProtocolTest, RawVelocityToRadSZero)
{
    EXPECT_DOUBLE_EQ(protocol::raw_velocity_to_rad_s(0), 0.0);
}

TEST_F(ST3215ProtocolTest, RawVelocityToRadSFullScaleIsServoMaxRpm)
{
    // Full-scale magnitude represents servo_max_rpm
    const double result = protocol::raw_velocity_to_rad_s(protocol::MAX_VELOCITY_PROTOCOL, 480.0);
    EXPECT_NEAR(result, 480.0 * 2.0 * std::numbers::pi / 60.0, 1e-9);
}

TEST_F(ST3215ProtocolTest, RawVelocityToRadSDefaultScaleIsAboutOneStepPerSecond)
{
    // One protocol unit is ~one encoder step per second (2*pi/4096 rad/s);
    // the calibrated default is within 2% of that nominal value.
    const double one_step_per_second = 2.0 * std::numbers::pi / 4096.0;
    EXPECT_NEAR(protocol::raw_velocity_to_rad_s(4096) / (4096 * one_step_per_second), 1.0, 0.02);
}

TEST_F(ST3215ProtocolTest, RawVelocityToRadSNegative)
{
    EXPECT_NEAR(protocol::raw_velocity_to_rad_s(-500), -protocol::raw_velocity_to_rad_s(500), 1e-12);
    EXPECT_LT(protocol::raw_velocity_to_rad_s(-500), 0.0);
}

TEST_F(ST3215ProtocolTest, EncodeServoVelocityZero)
{
    EXPECT_EQ(protocol::encode_servo_velocity(0.0), 0u);
    // Negative zero / tiny values must not produce a bare sign bit
    EXPECT_EQ(protocol::encode_servo_velocity(-1e-9), 0u);
}

TEST_F(ST3215ProtocolTest, EncodeServoVelocityUsesServoMaxRpm)
{
    // 1 rad/s = 9.549 RPM; with full scale = 480 RPM -> 9.549 * 32767 / 480 = 651.9
    EXPECT_EQ(protocol::encode_servo_velocity(1.0, 480.0), 652u);
}

TEST_F(ST3215ProtocolTest, EncodeServoVelocityDriveSpeedIsNotClamped)
{
    // Regression: the old scale clamped every wheel at ~0.79 rad/s
    // (~0.06 m/s at r=0.0725). A typical drive speed of 5 rad/s must encode
    // without hitting the protocol limit.
    const uint16_t result = protocol::encode_servo_velocity(5.0);
    EXPECT_GT(result, 3000u);
    EXPECT_LT(result, static_cast<uint16_t>(protocol::MAX_VELOCITY_PROTOCOL));
}

TEST_F(ST3215ProtocolTest, EncodeServoVelocityNegative)
{
    uint16_t result = protocol::encode_servo_velocity(-0.1);
    // Should have sign bit set (bit 15)
    EXPECT_TRUE(result & 0x8000);
    // Magnitude should match positive case
    uint16_t magnitude = result & 0x7FFF;
    uint16_t positive_result = protocol::encode_servo_velocity(0.1);
    EXPECT_EQ(magnitude, positive_result);
}

TEST_F(ST3215ProtocolTest, EncodeServoVelocityClampPositive)
{
    uint16_t result = protocol::encode_servo_velocity(1000.0);
    EXPECT_EQ(result, static_cast<uint16_t>(protocol::MAX_VELOCITY_PROTOCOL));
}

TEST_F(ST3215ProtocolTest, EncodeServoVelocityClampNegative)
{
    uint16_t result = protocol::encode_servo_velocity(-1000.0);
    uint16_t magnitude = result & 0x7FFF;
    EXPECT_EQ(magnitude, static_cast<uint16_t>(protocol::MAX_VELOCITY_PROTOCOL));
    EXPECT_TRUE(result & 0x8000);
}

TEST_F(ST3215ProtocolTest, EncodeServoVelocityRejectsNonFinite)
{
    EXPECT_EQ(protocol::encode_servo_velocity(std::nan("")), 0u);
    EXPECT_EQ(protocol::encode_servo_velocity(std::numeric_limits<double>::infinity()), 0u);
    EXPECT_EQ(protocol::encode_servo_velocity(1.0, 0.0), 0u);
}

TEST_F(ST3215ProtocolTest, PositionDeltaWithoutWrap)
{
    EXPECT_EQ(protocol::position_delta_ticks(100, 300), 200);
    EXPECT_EQ(protocol::position_delta_ticks(300, 100), -200);
    EXPECT_EQ(protocol::position_delta_ticks(1234, 1234), 0);
}

TEST_F(ST3215ProtocolTest, PositionDeltaWrapsForward)
{
    // 4090 -> 10 is +16 ticks across the 4095/0 boundary, not -4080
    EXPECT_EQ(protocol::position_delta_ticks(4090, 10), 16);
}

TEST_F(ST3215ProtocolTest, PositionDeltaWrapsBackward)
{
    EXPECT_EQ(protocol::position_delta_ticks(10, 4090), -16);
}

TEST_F(ST3215ProtocolTest, PositionDeltaAccumulatesAcrossRevolutions)
{
    // Simulate two full forward revolutions sampled every 300 ticks
    uint16_t raw = 4000;
    int64_t total = 0;
    for (int step = 0; step < 2 * 4096 / 300; ++step)
    {
        const auto next = static_cast<uint16_t>((raw + 300) % 4096);
        total += protocol::position_delta_ticks(raw, next);
        raw = next;
    }
    EXPECT_EQ(total, static_cast<int64_t>(2 * 4096 / 300) * 300);
}

// Default rover wiring: left-side servos (IDs 2 and 3) are inverted
const std::array<uint8_t, 2> kLeftSideInverted{2, 3};

TEST_F(ST3215ProtocolTest, MotorDirectionRightSide)
{
    // Servo IDs 1 and 4 are right-side, no inversion
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(1, 1.0, kLeftSideInverted), 1.0);
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(4, 1.0, kLeftSideInverted), 1.0);
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(1, -1.0, kLeftSideInverted), -1.0);
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(4, -1.0, kLeftSideInverted), -1.0);
}

TEST_F(ST3215ProtocolTest, MotorDirectionLeftSide)
{
    // Servo IDs 2 and 3 are left-side, direction is inverted
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(2, 1.0, kLeftSideInverted), -1.0);
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(3, 1.0, kLeftSideInverted), -1.0);
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(2, -1.0, kLeftSideInverted), 1.0);
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(3, -1.0, kLeftSideInverted), 1.0);
}

TEST_F(ST3215ProtocolTest, MotorDirectionZero)
{
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(2, 0.0, kLeftSideInverted), 0.0);
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(1, 0.0, kLeftSideInverted), 0.0);
}

TEST_F(ST3215ProtocolTest, MotorDirectionCustomInversionList)
{
    const std::array<uint8_t, 2> custom{1, 4};
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(1, 1.0, custom), -1.0);
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(2, 1.0, custom), 1.0);
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(3, 1.0, custom), 1.0);
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(4, 1.0, custom), -1.0);
}

TEST_F(ST3215ProtocolTest, MotorDirectionEmptyInversionList)
{
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(2, 1.0, {}), 1.0);
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(3, -1.0, {}), -1.0);
}

TEST_F(ST3215ProtocolTest, VelocityRoundTrip)
{
    // Encode then decode should produce approximately the same value
    // for velocities within the representable range
    double input = 0.3;
    uint16_t encoded = protocol::encode_servo_velocity(input);

    // Decode: strip sign bit, convert back
    int16_t raw;
    if (encoded & 0x8000)
    {
        raw = -static_cast<int16_t>(encoded & 0x7FFF);
    }
    else
    {
        raw = static_cast<int16_t>(encoded);
    }

    double decoded = protocol::raw_velocity_to_rad_s(raw);

    // Round-trip error comes from integer rounding in encode
    EXPECT_NEAR(decoded, input, 0.002);
}

TEST_F(ST3215ProtocolTest, VelocityRoundTripNegative)
{
    double input = -0.5;
    uint16_t encoded = protocol::encode_servo_velocity(input);

    int16_t raw;
    if (encoded & 0x8000)
    {
        raw = -static_cast<int16_t>(encoded & 0x7FFF);
    }
    else
    {
        raw = static_cast<int16_t>(encoded);
    }

    double decoded = protocol::raw_velocity_to_rad_s(raw);
    EXPECT_NEAR(decoded, input, 0.002);
}

TEST_F(ST3215ProtocolTest, BuildPacketAllServoIds)
{
    const std::array<uint8_t, 2> data{0x38, 0x08};
    for (uint8_t id : {1, 2, 3, 4})
    {
        auto packet = protocol::build_packet(id, protocol::Command::READ, data);
        EXPECT_EQ(packet[2], id) << "Packet ID mismatch for servo " << static_cast<int>(id);
    }
}

TEST_F(ST3215ProtocolTest, ParseSignedValueMaxPositive)
{
    // Maximum positive value (all bits except sign bit)
    EXPECT_EQ(protocol::parse_signed_value(0x7FFF), 0x7FFF);
}

TEST_F(ST3215ProtocolTest, ParseSignedValueMaxNegative)
{
    // Maximum negative value: sign bit | 0x7FFF
    EXPECT_EQ(protocol::parse_signed_value(0xFFFF), -0x7FFF);
}

namespace
{
    // Builds a wire-format status response: FF FF ID LEN ERROR <8 data bytes> SUM
    std::vector<uint8_t> make_status_response(uint8_t id, uint8_t error,
                                              uint16_t raw_pos, uint16_t raw_vel,
                                              uint8_t voltage, uint8_t temperature)
    {
        std::vector<uint8_t> payload{
            error,
            static_cast<uint8_t>(raw_pos & 0xFF),
            static_cast<uint8_t>(raw_pos >> 8),
            static_cast<uint8_t>(raw_vel & 0xFF),
            static_cast<uint8_t>(raw_vel >> 8),
            0x00,  // load low
            0x00,  // load high
            voltage,
            temperature,
        };

        std::vector<uint8_t> packet{0xFF, 0xFF, id,
                                    static_cast<uint8_t>(payload.size() + 1)};
        packet.insert(packet.end(), payload.begin(), payload.end());
        packet.push_back(protocol::calculate_checksum(
            std::span{packet.data() + 2, packet.size() - 2}));
        return packet;
    }
}  // namespace

TEST_F(ST3215ProtocolTest, ExtractPacketsSingleValidPacket)
{
    auto response = make_status_response(3, 0, 1024, 100, 120, 32);
    auto packets = protocol::extract_packets(response);

    ASSERT_EQ(packets.size(), 1u);
    EXPECT_EQ(packets[0].id, 3);
    EXPECT_EQ(packets[0].payload.size(), 9u);  // error byte + 8 data bytes
}

TEST_F(ST3215ProtocolTest, ExtractPacketsEmptyAndTooShortBuffers)
{
    EXPECT_TRUE(protocol::extract_packets({}).empty());

    const std::array<uint8_t, 3> too_short{0xFF, 0xFF, 0x01};
    EXPECT_TRUE(protocol::extract_packets(too_short).empty());
}

TEST_F(ST3215ProtocolTest, ExtractPacketsGarbagePrefixIsSkipped)
{
    auto response = make_status_response(2, 0, 512, 0, 120, 30);
    std::vector<uint8_t> noisy{0x12, 0x00, 0xFF, 0x55};
    noisy.insert(noisy.end(), response.begin(), response.end());

    auto packets = protocol::extract_packets(noisy);
    ASSERT_EQ(packets.size(), 1u);
    EXPECT_EQ(packets[0].id, 2);
}

TEST_F(ST3215ProtocolTest, ExtractPacketsBadChecksumIsRejected)
{
    auto response = make_status_response(1, 0, 100, 100, 120, 30);
    response.back() ^= 0xFF;  // corrupt the checksum

    EXPECT_TRUE(protocol::extract_packets(response).empty());
}

TEST_F(ST3215ProtocolTest, ExtractPacketsTruncatedPacketIsRejected)
{
    auto response = make_status_response(1, 0, 100, 100, 120, 30);
    response.resize(response.size() - 3);  // drop the tail mid-payload

    EXPECT_TRUE(protocol::extract_packets(response).empty());
}

TEST_F(ST3215ProtocolTest, ExtractPacketsMultiplePacketsInOneBuffer)
{
    auto first = make_status_response(1, 0, 100, 0, 120, 25);
    auto second = make_status_response(4, 0, 200, 50, 121, 26);
    std::vector<uint8_t> buffer = first;
    buffer.insert(buffer.end(), second.begin(), second.end());

    auto packets = protocol::extract_packets(buffer);
    ASSERT_EQ(packets.size(), 2u);
    EXPECT_EQ(packets[0].id, 1);
    EXPECT_EQ(packets[1].id, 4);
}

TEST_F(ST3215ProtocolTest, ExtractPacketsZeroLengthIsRejected)
{
    // A length byte of zero cannot carry a checksum
    const std::array<uint8_t, 5> packet{0xFF, 0xFF, 0x01, 0x00, 0xFE};
    EXPECT_TRUE(protocol::extract_packets(packet).empty());
}

TEST_F(ST3215ProtocolTest, ParseStatusPayloadDecodesFields)
{
    auto response = make_status_response(3, 0, 2048, 0x8064 /* -100 */, 120, 32);
    auto packets = protocol::extract_packets(response);
    ASSERT_EQ(packets.size(), 1u);

    auto status = protocol::parse_status_payload(packets[0].payload);
    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(status->error_flags, 0);
    EXPECT_NEAR(status->position_rad, std::numbers::pi, 1e-10);
    EXPECT_EQ(status->raw_position, 2048);
    EXPECT_NEAR(status->velocity_rad_s, protocol::raw_velocity_to_rad_s(-100), 1e-10);
    EXPECT_DOUBLE_EQ(status->temperature_c, 32.0);
}

TEST_F(ST3215ProtocolTest, ParseStatusPayloadReadsTemperatureNotVoltage)
{
    // Regression: temperature is the 8th data byte (reg 0x3F); the 7th is
    // voltage (reg 0x3E) and must not be reported as temperature
    auto response = make_status_response(1, 0, 0, 0, /*voltage=*/120, /*temperature=*/41);
    auto packets = protocol::extract_packets(response);
    ASSERT_EQ(packets.size(), 1u);

    auto status = protocol::parse_status_payload(packets[0].payload);
    ASSERT_TRUE(status.has_value());
    EXPECT_DOUBLE_EQ(status->temperature_c, 41.0);
}

TEST_F(ST3215ProtocolTest, ParseStatusPayloadPropagatesErrorFlags)
{
    auto response = make_status_response(1, 0x24 /* overheating | overload */, 0, 0, 120, 85);
    auto packets = protocol::extract_packets(response);
    ASSERT_EQ(packets.size(), 1u);

    auto status = protocol::parse_status_payload(packets[0].payload);
    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(status->error_flags, 0x24);
}

TEST_F(ST3215ProtocolTest, ParseStatusPayloadTooShortReturnsNullopt)
{
    const std::array<uint8_t, 3> short_payload{0x00, 0x01, 0x02};
    EXPECT_FALSE(protocol::parse_status_payload(short_payload).has_value());
    EXPECT_FALSE(protocol::parse_status_payload({}).has_value());
}
