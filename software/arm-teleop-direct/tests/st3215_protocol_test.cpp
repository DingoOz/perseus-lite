// Unit tests for the pure ST3215 protocol helpers (st3215-protocol.hpp).
// Dependency-free: run with `ctest` from the build directory.

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "st3215-protocol.hpp"

using namespace perseus::servo;

namespace
{
    int failures = 0;

    void check(bool condition, const char* expression, int line)
    {
        if (!condition)
        {
            std::cerr << "FAILED (line " << line << "): " << expression << "\n";
            ++failures;
        }
    }

    template <typename F>
    bool throws_protocol_error(F&& f)
    {
        try
        {
            f();
        }
        catch (const st3215::ProtocolError&)
        {
            return true;
        }
        return false;
    }
}  // namespace

#define CHECK(expr) check((expr), #expr, __LINE__)

int main()
{
    // Checksum = ~(ID + LEN + INSTR + params) & 0xFF.
    // Read 2 bytes from 0x38 on servo 1: FF FF 01 04 02 38 02 BE
    const std::vector<uint8_t> read_cmd = {0x01, 0x04, 0x02, 0x38, 0x02};
    CHECK(st3215::checksum(read_cmd) == 0xBE);

    // Valid status reply: FF FF 01 04 00 E8 03 0F (position 1000, no error)
    const std::vector<uint8_t> good_body = {0x00, 0xE8, 0x03, 0x0F};
    const auto packet = st3215::parse_status_body(0x01, good_body);
    CHECK(packet.id == 0x01);
    CHECK(packet.error == 0x00);
    CHECK(packet.params.size() == 2);
    CHECK(st3215::word_le(packet.params[0], packet.params[1]) == 1000);

    // Corrupted parameter byte -> checksum mismatch is rejected
    const std::vector<uint8_t> bad_body = {0x00, 0xE9, 0x03, 0x0F};
    CHECK(throws_protocol_error([&]
                                { (void)st3215::parse_status_body(0x01, bad_body); }));
    // Same body attributed to another ID also fails the checksum
    CHECK(throws_protocol_error([&]
                                { (void)st3215::parse_status_body(0x02, good_body); }));
    // Too short (LEN must cover at least ERR + CHK)
    const std::vector<uint8_t> short_body = {0x00};
    CHECK(throws_protocol_error([&]
                                { (void)st3215::parse_status_body(0x01, short_body); }));

    // Error byte is surfaced and formatted (main.cpp matches on "Overload")
    // FF FF 03 02 20 CHK: ~(3 + 2 + 0x20) = ~0x25 = 0xDA
    const std::vector<uint8_t> overload_body = {0x20, 0xDA};
    const auto overload = st3215::parse_status_body(0x03, overload_body);
    CHECK(overload.error == error_bits::OVERLOAD);
    CHECK(overload.params.empty());
    CHECK(st3215::format_error(overload.error).find("Overload") != std::string::npos);

    // Load is sign-magnitude with the direction in bit 10, NOT two's
    // complement: 0x0464 is -100 (a two's-complement decode gave +1124,
    // which tripped the 800 torque-protection threshold).
    CHECK(st3215::decode_load(0x0064) == 100);
    CHECK(st3215::decode_load(0x0464) == -100);
    CHECK(st3215::decode_load(0x0400 | 1000) == -1000);
    CHECK(st3215::decode_load(0x0400) == 0);
    CHECK(st3215::decode_load(1000) == 1000);

    // Speed: direction in bit 15
    CHECK(st3215::decode_speed(500) == 500);
    CHECK(st3215::decode_speed(0x8000 | 500) == -500);

    // Position: direction in bit 15; clamped to the 0-4095 position range
    CHECK(st3215::decode_position(2048) == 2048);
    CHECK(st3215::decode_position(0x8000 | 5) == 0);
    CHECK(st3215::decode_position(5000) == limits::MAX_POSITION);

    if (failures != 0)
    {
        std::cerr << failures << " check(s) failed\n";
        return EXIT_FAILURE;
    }
    std::cout << "All ST3215 protocol checks passed\n";
    return EXIT_SUCCESS;
}
