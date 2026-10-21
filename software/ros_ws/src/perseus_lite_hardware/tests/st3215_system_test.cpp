// System-level tests for ST3215SystemHardware: the real plugin talks to a fake
// servo bus on a pseudo-terminal, so lifecycle, direction handling, feedback
// staleness and unwrapping are exercised end to end without hardware.

#include "st3215_system.hpp"

#include <fcntl.h>
#include <gtest/gtest.h>
#include <poll.h>
#include <pty.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/types/hardware_component_params.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <map>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "st3215_protocol.hpp"

namespace protocol = perseus_lite_hardware::protocol;
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using namespace std::chrono_literals;

namespace
{
    constexpr uint8_t REG_TORQUE_ENABLE = 40;
    constexpr uint8_t REG_ACC = 41;
    constexpr uint8_t REG_GOAL_SPEED_L = 46;
    constexpr uint8_t REG_MODE = 33;
    constexpr uint8_t REG_PRESENT_POSITION_L = 0x38;

    struct RegisterWrite
    {
        uint8_t reg = 0;
        std::vector<uint8_t> data;
    };

    // Emulates the servos' side of the half-duplex bus: records WRITEs,
    // acknowledges them (status return level 1) and answers status READs.
    class FakeServoBus
    {
    public:
        FakeServoBus()
        {
            char name[128] = {};
            if (openpty(&_master, &_slave, name, nullptr, nullptr) != 0)
            {
                throw std::runtime_error(std::string("openpty failed: ") + std::strerror(errno));
            }
            _slave_name = name;
            _thread = std::thread([this]
                                  { run(); });
        }

        ~FakeServoBus()
        {
            _running = false;
            _thread.join();
            ::close(_slave);
            ::close(_master);
        }

        const std::string& port() const { return _slave_name; }

        void set_feedback(uint8_t id, uint16_t raw_position, int16_t speed_units)
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _raw_position[id] = raw_position;
            _speed[id] = speed_units;
        }

        // A servo that acknowledges writes but never answers status reads
        void set_ack_only(uint8_t id)
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _ack_only.insert(id);
        }

        void set_silent(uint8_t id)
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _silent.insert(id);
        }

        std::vector<RegisterWrite> writes(uint8_t id)
        {
            std::lock_guard<std::mutex> lock(_mutex);
            return _writes[id];
        }

        void clear_writes()
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _writes.clear();
        }

    private:
        void send(uint8_t id, const std::vector<uint8_t>& payload)
        {
            std::vector<uint8_t> packet{0xFF, 0xFF, id, static_cast<uint8_t>(payload.size() + 1)};
            packet.insert(packet.end(), payload.begin(), payload.end());
            packet.push_back(protocol::calculate_checksum(std::span{packet.data() + 2, packet.size() - 2}));
            ssize_t ignored = ::write(_master, packet.data(), packet.size());
            (void)ignored;
        }

        void handle(uint8_t id, uint8_t cmd, std::span<const uint8_t> params)
        {
            std::lock_guard<std::mutex> lock(_mutex);
            if (_silent.count(id))
            {
                return;
            }
            if (cmd == static_cast<uint8_t>(protocol::Command::WRITE) && !params.empty())
            {
                _writes[id].push_back({params[0], {params.begin() + 1, params.end()}});
                send(id, {0x00});  // ACK: error byte only
            }
            else if (cmd == static_cast<uint8_t>(protocol::Command::READ) && params.size() == 2 &&
                     params[0] == REG_PRESENT_POSITION_L && !_ack_only.count(id))
            {
                const uint16_t pos = _raw_position[id];
                const int16_t speed = _speed[id];
                const uint16_t raw_speed = speed < 0
                                               ? static_cast<uint16_t>(static_cast<uint16_t>(-speed) | protocol::SIGN_BIT_MASK)
                                               : static_cast<uint16_t>(speed);
                send(id, {0x00,
                          static_cast<uint8_t>(pos & 0xFF), static_cast<uint8_t>(pos >> 8),
                          static_cast<uint8_t>(raw_speed & 0xFF), static_cast<uint8_t>(raw_speed >> 8),
                          0x00, 0x00, /*voltage*/ 120, /*temperature*/ 31});
            }
        }

        void run()
        {
            std::vector<uint8_t> buffer;
            while (_running)
            {
                pollfd pfd{_master, POLLIN, 0};
                if (::poll(&pfd, 1, 5) <= 0 || !(pfd.revents & POLLIN))
                {
                    continue;
                }
                uint8_t chunk[256];
                const ssize_t n = ::read(_master, chunk, sizeof(chunk));
                if (n <= 0)
                {
                    continue;
                }
                buffer.insert(buffer.end(), chunk, chunk + n);

                // Host packets: FF FF ID LEN CMD PARAMS... SUM (LEN = params + 2)
                size_t i = 0;
                while (i + 4 <= buffer.size())
                {
                    if (buffer[i] != 0xFF || buffer[i + 1] != 0xFF)
                    {
                        ++i;
                        continue;
                    }
                    const uint8_t len = buffer[i + 3];
                    if (i + 4 + len > buffer.size())
                    {
                        break;
                    }
                    const uint8_t sum = protocol::calculate_checksum(std::span{buffer.data() + i + 2, static_cast<size_t>(len + 1)});
                    if (len >= 2 && sum == buffer[i + 3 + len])
                    {
                        handle(buffer[i + 2], buffer[i + 4],
                               std::span{buffer.data() + i + 5, static_cast<size_t>(len - 2)});
                    }
                    i += 4 + len;
                }
                buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(i));
            }
        }

        int _master{-1};
        int _slave{-1};
        std::string _slave_name;
        std::atomic<bool> _running{true};
        std::thread _thread;
        std::mutex _mutex;
        std::map<uint8_t, uint16_t> _raw_position;
        std::map<uint8_t, int16_t> _speed;
        std::set<uint8_t> _ack_only;
        std::set<uint8_t> _silent;
        std::map<uint8_t, std::vector<RegisterWrite>> _writes;
    };

    std::string make_urdf(const std::string& port, const std::string& baud = "1000000")
    {
        auto joint = [](const std::string& name, int id)
        {
            return "<joint name=\"" + name +
                   "\">"
                   "<command_interface name=\"velocity\"/>"
                   "<state_interface name=\"position\"/>"
                   "<state_interface name=\"velocity\"/>"
                   "<state_interface name=\"temperature\"/>"
                   "<param name=\"id\">" +
                   std::to_string(id) + "</param></joint>";
        };
        std::string kinematics = "<link name=\"base_link\"/>";
        for (const char* name : {"front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"})
        {
            kinematics += std::string("<link name=\"") + name +
                          "\"/>"
                          "<joint name=\"" +
                          name +
                          "_joint\" type=\"continuous\"><parent link=\"base_link\"/>"
                          "<child link=\"" +
                          name + "\"/><axis xyz=\"0 1 0\"/></joint>";
        }
        return "<?xml version=\"1.0\"?><robot name=\"test\">" + kinematics +
               "<ros2_control name=\"Perseus_Lite\" type=\"system\"><hardware>"
               "<plugin>perseus_lite_hardware/ST3215SystemHardware</plugin>"
               "<param name=\"serial_port\">" +
               port + "</param><param name=\"baud_rate\">" + baud +
               "</param>"
               "<param name=\"servo_max_rpm\">480.0</param>"
               "</hardware>" +
               joint("front_left_wheel_joint", 3) + joint("front_right_wheel_joint", 4) +
               joint("rear_left_wheel_joint", 2) + joint("rear_right_wheel_joint", 1) +
               "</ros2_control></robot>";
    }

    // Joint order matches make_urdf()
    constexpr std::array<uint8_t, 4> kJointIds{3, 4, 2, 1};
    constexpr size_t FL = 0, FR = 1, RL = 2, RR = 3;

    std::optional<RegisterWrite> last_write_to(const std::vector<RegisterWrite>& writes, uint8_t reg)
    {
        for (auto it = writes.rbegin(); it != writes.rend(); ++it)
        {
            if (it->reg == reg)
            {
                return *it;
            }
        }
        return std::nullopt;
    }

    uint16_t as_u16(const RegisterWrite& w)
    {
        return static_cast<uint16_t>(w.data.at(0) | (w.data.at(1) << 8));
    }
}  // namespace

class ST3215SystemTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        if (!rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }
    }

    CallbackReturn init_hardware(const std::string& baud = "1000000")
    {
        auto infos = hardware_interface::parse_control_resources_from_urdf(make_urdf(bus.port(), baud));
        hardware_interface::HardwareComponentParams params;
        params.hardware_info = infos.at(0);
        // The controller manager hands hardware a steady clock on real robots;
        // mixing it with ROS-time stamps used to throw on the first read().
        params.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
        params.logger = rclcpp::get_logger("st3215_system_test");
        const auto result = hw.init(params);
        if (result == CallbackReturn::SUCCESS)
        {
            states = hw.export_state_interfaces();
            commands = hw.export_command_interfaces();
        }
        return result;
    }

    double state(size_t joint, const std::string& interface)
    {
        for (const auto& s : states)
        {
            if (s.get_prefix_name() == hw.get_hardware_info().joints[joint].name && s.get_interface_name() == interface)
            {
                return s.get_optional<double>().value();
            }
        }
        throw std::runtime_error("no state interface");
    }

    void command_all(double rad_s)
    {
        for (auto& c : commands)
        {
            ASSERT_TRUE(c.set_value(rad_s));
        }
    }

    // Let the communication thread complete a few polling cycles
    static void wait_for_polls() { std::this_thread::sleep_for(300ms); }

    FakeServoBus bus;
    perseus_lite_hardware::ST3215SystemHardware hw;
    std::vector<hardware_interface::StateInterface> states;
    std::vector<hardware_interface::CommandInterface> commands;
};

TEST_F(ST3215SystemTest, ConfigureZeroesSpeedBeforeEnablingTorque)
{
    ASSERT_EQ(init_hardware(), CallbackReturn::SUCCESS);
    ASSERT_EQ(hw.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

    for (uint8_t id : kJointIds)
    {
        const auto writes = bus.writes(id);
        ASSERT_GE(writes.size(), 4u) << "servo " << int(id);
        EXPECT_EQ(writes[0].reg, REG_GOAL_SPEED_L);
        EXPECT_EQ(as_u16(writes[0]), 0u);
        EXPECT_EQ(writes[1].reg, REG_MODE);
        EXPECT_EQ(writes[1].data.at(0), 1u);  // wheel mode
        EXPECT_EQ(writes[2].reg, REG_ACC);
        EXPECT_EQ(writes[3].reg, REG_TORQUE_ENABLE);
        EXPECT_EQ(writes[3].data.at(0), 1u);
    }
    EXPECT_EQ(hw.on_cleanup(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(ST3215SystemTest, WriteInvertsLeftSideServos)
{
    ASSERT_EQ(init_hardware(), CallbackReturn::SUCCESS);
    ASSERT_EQ(hw.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    ASSERT_EQ(hw.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

    command_all(1.0);
    ASSERT_EQ(hw.write(rclcpp::Time(), rclcpp::Duration(0, 0)), return_type::OK);
    std::this_thread::sleep_for(50ms);

    const uint16_t forward = protocol::encode_servo_velocity(1.0, 480.0);
    for (uint8_t id : {uint8_t{1}, uint8_t{4}})
    {
        const auto w = last_write_to(bus.writes(id), REG_GOAL_SPEED_L);
        ASSERT_TRUE(w) << "servo " << int(id);
        EXPECT_EQ(as_u16(*w), forward) << "right-side servo " << int(id);
    }
    for (uint8_t id : {uint8_t{2}, uint8_t{3}})
    {
        const auto w = last_write_to(bus.writes(id), REG_GOAL_SPEED_L);
        ASSERT_TRUE(w) << "servo " << int(id);
        EXPECT_EQ(as_u16(*w), forward | protocol::SIGN_BIT_MASK) << "left-side servo " << int(id);
    }
    EXPECT_EQ(hw.on_cleanup(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(ST3215SystemTest, ReadInvertsLeftSideFeedback)
{
    // Driving forward: the mirrored left servos physically report the
    // opposite sign; every joint must read forward after inversion.
    bus.set_feedback(1, 0, 1000);
    bus.set_feedback(4, 0, 1000);
    bus.set_feedback(2, 0, -1000);
    bus.set_feedback(3, 0, -1000);

    ASSERT_EQ(init_hardware(), CallbackReturn::SUCCESS);
    ASSERT_EQ(hw.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    wait_for_polls();
    ASSERT_EQ(hw.read(rclcpp::Time(), rclcpp::Duration(0, 0)), return_type::OK);

    const double expected = protocol::raw_velocity_to_rad_s(1000, 480.0);
    for (size_t joint : {FL, FR, RL, RR})
    {
        EXPECT_NEAR(state(joint, hardware_interface::HW_IF_VELOCITY), expected, 1e-9)
            << hw.get_hardware_info().joints[joint].name;
    }
    EXPECT_DOUBLE_EQ(state(FR, hardware_interface::HW_IF_TEMPERATURE), 31.0);
    EXPECT_EQ(hw.on_cleanup(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(ST3215SystemTest, PositionUnwrapsAcrossRevolutionBoundary)
{
    bus.set_feedback(1, 4000, 0);
    ASSERT_EQ(init_hardware(), CallbackReturn::SUCCESS);
    ASSERT_EQ(hw.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    wait_for_polls();
    ASSERT_EQ(hw.read(rclcpp::Time(), rclcpp::Duration(0, 0)), return_type::OK);
    const double before = state(RR, hardware_interface::HW_IF_POSITION);

    // Wheel moves forward 196 ticks, wrapping 4095 -> 0
    bus.set_feedback(1, 100, 0);
    wait_for_polls();
    ASSERT_EQ(hw.read(rclcpp::Time(), rclcpp::Duration(0, 0)), return_type::OK);
    const double after = state(RR, hardware_interface::HW_IF_POSITION);

    EXPECT_NEAR(after - before, protocol::ticks_to_radians(196), 1e-9);
    EXPECT_EQ(hw.on_cleanup(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(ST3215SystemTest, WriteAcknowledgementsDoNotCountAsFeedback)
{
    for (uint8_t id : kJointIds)
    {
        bus.set_feedback(id, 0, 500);
    }
    bus.set_ack_only(4);

    ASSERT_EQ(init_hardware(), CallbackReturn::SUCCESS);
    ASSERT_EQ(hw.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    ASSERT_EQ(hw.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

    // Keep writing (each write is ACKed by every servo) past the 1 s timeout
    const auto end = std::chrono::steady_clock::now() + 1300ms;
    while (std::chrono::steady_clock::now() < end)
    {
        ASSERT_EQ(hw.write(rclcpp::Time(), rclcpp::Duration(0, 0)), return_type::OK);
        std::this_thread::sleep_for(20ms);
    }
    ASSERT_EQ(hw.read(rclcpp::Time(), rclcpp::Duration(0, 0)), return_type::OK);

    EXPECT_DOUBLE_EQ(state(FR, hardware_interface::HW_IF_VELOCITY), 0.0) << "ACK-only servo must read as stale";
    EXPECT_GT(std::abs(state(RR, hardware_interface::HW_IF_VELOCITY)), 0.0);
    EXPECT_EQ(hw.on_cleanup(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(ST3215SystemTest, MissingServoDoesNotBlockConfigure)
{
    bus.set_silent(2);
    ASSERT_EQ(init_hardware(), CallbackReturn::SUCCESS);

    const auto start = std::chrono::steady_clock::now();
    EXPECT_EQ(hw.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    EXPECT_LT(std::chrono::steady_clock::now() - start, 2s);

    wait_for_polls();
    ASSERT_EQ(hw.read(rclcpp::Time(), rclcpp::Duration(0, 0)), return_type::OK);
    EXPECT_DOUBLE_EQ(state(RL, hardware_interface::HW_IF_VELOCITY), 0.0);
    EXPECT_EQ(hw.on_cleanup(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(ST3215SystemTest, CleanupStopsServosAndDisablesTorque)
{
    ASSERT_EQ(init_hardware(), CallbackReturn::SUCCESS);
    ASSERT_EQ(hw.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    ASSERT_EQ(hw.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    command_all(2.0);
    ASSERT_EQ(hw.write(rclcpp::Time(), rclcpp::Duration(0, 0)), return_type::OK);

    bus.clear_writes();
    ASSERT_EQ(hw.on_cleanup(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    std::this_thread::sleep_for(50ms);

    for (uint8_t id : kJointIds)
    {
        const auto writes = bus.writes(id);
        const auto speed = last_write_to(writes, REG_GOAL_SPEED_L);
        const auto torque = last_write_to(writes, REG_TORQUE_ENABLE);
        ASSERT_TRUE(speed && torque) << "servo " << int(id);
        EXPECT_EQ(as_u16(*speed), 0u);
        EXPECT_EQ(torque->data.at(0), 0u);
    }
}

TEST_F(ST3215SystemTest, ErrorTransitionStopsServos)
{
    ASSERT_EQ(init_hardware(), CallbackReturn::SUCCESS);
    ASSERT_EQ(hw.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    ASSERT_EQ(hw.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    command_all(2.0);
    ASSERT_EQ(hw.write(rclcpp::Time(), rclcpp::Duration(0, 0)), return_type::OK);

    bus.clear_writes();
    EXPECT_EQ(hw.on_error(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    std::this_thread::sleep_for(50ms);

    for (uint8_t id : kJointIds)
    {
        const auto speed = last_write_to(bus.writes(id), REG_GOAL_SPEED_L);
        ASSERT_TRUE(speed) << "servo " << int(id);
        EXPECT_EQ(as_u16(*speed), 0u);
    }
}

TEST_F(ST3215SystemTest, MalformedBaudRateFailsConfigureWithoutThrowing)
{
    ASSERT_EQ(init_hardware("fast"), CallbackReturn::SUCCESS);
    CallbackReturn result = CallbackReturn::SUCCESS;
    EXPECT_NO_THROW(result = hw.on_configure(rclcpp_lifecycle::State()));
    EXPECT_EQ(result, CallbackReturn::ERROR);
}

TEST_F(ST3215SystemTest, InvalidServoMaxRpmIsRejected)
{
    auto infos = hardware_interface::parse_control_resources_from_urdf(make_urdf(bus.port()));
    infos.at(0).hardware_parameters["servo_max_rpm"] = "-5";
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = infos.at(0);
    params.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    EXPECT_EQ(hw.init(params), CallbackReturn::ERROR);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
