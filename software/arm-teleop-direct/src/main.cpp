// Include guard to handle Boost.Asio before ncurses
#ifndef PERSEUS_ARM_ASIO_INCLUDES
#define PERSEUS_ARM_ASIO_INCLUDES
#include "perseus-arm-teleop.hpp"
#endif

#include <curses.h>
#include <signal.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "servo-constants.hpp"

// Use the servo constants namespace for easier access to constants
using namespace perseus::servo;

static WINDOW* ncurses_win = nullptr;

// Cleared by the SIGINT/SIGTERM/SIGHUP handler. The handler only stores to
// this flag (async-signal-safe because the atomic is lock-free); the main
// loop notices it, then disables torque and tears down ncurses itself.
static std::atomic<bool> running(true);
static_assert(std::atomic<bool>::is_always_lock_free, "signal flag must be lock-free");
static std::atomic<bool> torque_protection(false);  // Global flag for torque protection

// Thrown when a shutdown signal interrupts interactive start-up.
class ShutdownRequested : public std::runtime_error
{
public:
    ShutdownRequested()
        : std::runtime_error("Interrupted by signal")
    {
    }
};

// Find available serial ports
std::vector<std::string> find_serial_ports()
{
    std::vector<std::string> ports;
    const std::filesystem::path dev_path("/dev");
    // Overridable via LEADER_FOLLOWER_PORT (falls back to ~/leader_follower);
    // avoids hardcoding a specific developer's home directory.
    std::filesystem::path virtual_port_symlink;
    if (const char* env_port = std::getenv("LEADER_FOLLOWER_PORT"))
    {
        virtual_port_symlink = env_port;
    }
    else if (const char* home_dir = std::getenv("HOME"))
    {
        virtual_port_symlink = std::filesystem::path(home_dir) / "leader_follower";
    }

    // Check if the symlink to the virtual port exists and add it
    if (std::filesystem::exists(virtual_port_symlink))
    {
        // Add the symlink path as-is, since we want to use the symlink itself
        ports.push_back(virtual_port_symlink.string());
    }

    // Find physical serial ports
    for (const auto& entry : std::filesystem::directory_iterator(dev_path))
    {
        std::string filename = entry.path().filename().string();
        if (filename.find("ttyUSB") != std::string::npos ||
            filename.find("ttyACM") != std::string::npos)
        {
            ports.push_back(entry.path().string());
        }
    }

    std::sort(ports.begin(), ports.end());
    return ports;
}
// Let user select ports for both arms
std::pair<std::string, std::string> select_serial_ports(const std::vector<std::string>& ports)
{
    if (ports.empty())
    {
        throw std::runtime_error("No serial ports found");
    }

    std::cout << "\nSelect ports for Perseus arms control\n";
    std::cout << "=====================================\n";
    std::cout << "Available serial ports:\n";
    for (size_t i = 0; i < ports.size(); ++i)
    {
        std::cout << i + 1 << ": " << ports[i] << std::endl;
    }
    std::cout << "0: Rescan for ports\n\n";

    std::string port1, port2;
    size_t selection1 = 0, selection2 = 0;

    // Select first arm port
    while (true)
    {
        std::cout << "Select port for ARM 1 (0 to rescan, 1-" << ports.size() << " to select): ";
        if (!(std::cin >> selection1))
        {
            if (!running.load() || std::cin.eof())
            {
                throw ShutdownRequested();
            }
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            std::cout << "Invalid input. Please enter a number.\n";
            continue;
        }

        if (selection1 == 0)
        {
            return select_serial_ports(find_serial_ports());  // Rescan and restart
        }

        if (selection1 > 0 && selection1 <= ports.size())
        {
            port1 = ports[selection1 - 1];
            break;
        }

        std::cout << "Invalid selection. Please try again.\n";
    }

    // Display confirmation and proceed to second arm
    std::cout << "\nARM 1 will use: " << port1 << "\n\n";

    // Select second arm port
    while (true)
    {
        std::cout << "Select port for ARM 2 (0 to rescan, 1-" << ports.size() << " to select): ";
        if (!(std::cin >> selection2))
        {
            if (!running.load() || std::cin.eof())
            {
                throw ShutdownRequested();
            }
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            std::cout << "Invalid input. Please enter a number.\n";
            continue;
        }

        if (selection2 == 0)
        {
            return select_serial_ports(find_serial_ports());  // Rescan and restart
        }

        if (selection2 > 0 && selection2 <= ports.size() && selection2 != selection1)
        {
            port2 = ports[selection2 - 1];
            break;
        }

        if (selection2 == selection1)
        {
            std::cout << "Cannot use the same port for both arms. Please select a different port.\n";
        }
        else
        {
            std::cout << "Invalid selection. Please try again.\n";
        }
    }

    std::cout << "\nARM 2 will use: " << port2 << "\n";
    std::cout << "\nPress Enter to continue...";
    std::cin.ignore(10000, '\n');
    std::cin.get();
    if (!running.load())
    {
        throw ShutdownRequested();
    }

    return {port1, port2};
}

// Create a colored progress bar string
void display_progress_bar(WINDOW* win, int y, int x, uint16_t current, uint16_t min, uint16_t max)
{
    // Clamp values to 0-4095
    current = std::min(current, static_cast<uint16_t>(limits::MAX_POSITION));
    min = std::min(min, static_cast<uint16_t>(limits::MAX_POSITION));
    max = std::min(max, static_cast<uint16_t>(limits::MAX_POSITION));

    const size_t bar_length = ui::POSITION_BAR_LENGTH;

    // Calculate positions
    size_t current_pos = static_cast<size_t>((static_cast<double>(current) / limits::MAX_POSITION) * bar_length);
    size_t min_pos = static_cast<size_t>((static_cast<double>(min) / limits::MAX_POSITION) * bar_length);
    size_t max_pos = static_cast<size_t>((static_cast<double>(max) / limits::MAX_POSITION) * bar_length);

    // Print opening bracket
    mvwaddch(win, y, x, '[');
    x++;

    // Print bar with colors
    for (size_t i = 0; i < bar_length; i++)
    {
        if (has_colors())
        {
            if (i == min_pos)
            {
                wattron(win, COLOR_PAIR(1));  // Blue for min
                waddch(win, '#');
                wattroff(win, COLOR_PAIR(1));
            }
            else if (i == max_pos)
            {
                wattron(win, COLOR_PAIR(2));  // Green for max
                waddch(win, '#');
                wattroff(win, COLOR_PAIR(2));
            }
            else if (i < current_pos)
            {
                if (i < min_pos)
                {
                    wattron(win, COLOR_PAIR(3) | A_DIM);  // Dimmed white for positions before min
                    waddch(win, '#');
                    wattroff(win, COLOR_PAIR(3) | A_DIM);
                }
                else
                {
                    wattron(win, COLOR_PAIR(3));  // Bright white for current valid position
                    waddch(win, '#');
                    wattroff(win, COLOR_PAIR(3));
                }
            }
            else
            {
                waddch(win, ' ');
            }
        }
        else
        {
            // For non-color displays, still show all positions but with different characters
            if (i < current_pos)
            {
                waddch(win, (i < min_pos) ? '.' : '#');
            }
            else
            {
                waddch(win, ' ');
            }
        }
    }

    // Print closing bracket
    waddch(win, ']');
}

// Structure to hold servo data including min/max values
struct ServoData
{
    uint16_t current = 0;
    uint16_t min = limits::MAX_POSITION;
    uint16_t max = 0;
    int16_t torque = 0;  // New field for torque feedback
    std::string error;
    bool mirroring = false;  // Whether this servo is mirroring arm 1
};

// Create a colored torque bar string
void display_torque_bar(WINDOW* win, int y, int x, int16_t torque)
{
    const int width = ui::TORQUE_BAR_WIDTH;
    const int16_t max_display = ui::MAX_DISPLAY_TORQUE;  // Scale display to ±100

    // Scale torque to display width
    int scaled = static_cast<int>((std::abs(static_cast<float>(torque)) / max_display) * width);
    scaled = std::min(scaled, width);

    // Print opening bracket
    mvwaddch(win, y, x, '[');

    if (has_colors())
    {
        for (int i = 0; i < width; i++)
        {
            if (i < scaled)
            {
                int value = std::abs(torque);
                if (value >= 80)
                {
                    wattron(win, COLOR_PAIR(5));  // Red for high torque
                }
                else if (value >= 40)
                {
                    wattron(win, COLOR_PAIR(4));  // Yellow for medium torque
                }
                else
                {
                    wattron(win, COLOR_PAIR(6));  // Grey for low torque
                }
                waddch(win, torque >= 0 ? '+' : '-');
                wattroff(win, COLOR_PAIR(5) | COLOR_PAIR(4) | COLOR_PAIR(6));
            }
            else
            {
                waddch(win, ' ');
            }
        }
    }
    else
    {
        // Non-color display
        for (int i = 0; i < width; i++)
        {
            if (i < scaled)
            {
                waddch(win, torque >= 0 ? '+' : '-');
            }
            else
            {
                waddch(win, ' ');
            }
        }
    }

    // Print closing bracket
    waddch(win, ']');
}

// Scale a position value from one range to another
uint16_t scale_position(uint16_t pos, uint16_t src_min, uint16_t src_max, uint16_t dest_min, uint16_t dest_max)
{
    // Ensure we don't divide by zero
    if (src_max == src_min)
        return dest_min;

    // Calculate the scaling factor and apply it (in double, so a reversed or
    // out-of-range input can't wrap around the uint16_t result)
    const double scale = (static_cast<double>(dest_max) - dest_min) / (static_cast<double>(src_max) - src_min);
    const double scaled = dest_min + (static_cast<double>(pos) - src_min) * scale;
    return static_cast<uint16_t>(std::clamp(scaled, static_cast<double>(limits::MIN_POSITION),
                                            static_cast<double>(limits::MAX_POSITION)));
}

std::string get_working_directory()
{
    return std::filesystem::current_path().string();
}

// Display servo values in ncurses window for both arms
void display_servo_values(WINDOW* win,
                          const std::vector<ServoData>& arm1_data,
                          const std::vector<ServoData>& arm2_data)
{
    werase(win);

    // Display header
    mvwprintw(win, 0, 0, "Perseus Arms Servo Positions (0-4095) and Torque (-100 to +100)");
    mvwprintw(win, 1, 0, "--------------------------------------------------------");

    // Column headers
    mvwprintw(win, 2, 2, "Servo    Current    Min      Max      Torque  Range");
    mvwprintw(win, 3, 0, "--------------------------------------------------------");

    // Display first arm's servos
    mvwprintw(win, 4, 0, "Arm 1:");
    for (size_t i = 0; i < arm::SERVO_COUNT; ++i)
    {
        int row = i + 5;
        const auto& servo = arm1_data[i];

        if (servo.error.empty())
        {
            mvwprintw(win, row, 2, "%-8d %8u  %8u  %8u  [Torque: %5d] ",
                      static_cast<int>(i + 1),
                      servo.current,
                      servo.min,
                      servo.max,
                      servo.torque);

            // Display torque bar
            display_torque_bar(win, row, 42, servo.torque);

            // Display position bar
            display_progress_bar(win, row, 49, servo.current, servo.min, servo.max);
        }
        else
        {
            mvwprintw(win, row, 2, "%d: Error: %s",
                      static_cast<int>(i + 1),
                      servo.error.c_str());
        }
    }

    mvwprintw(win, 11, 0, "--------------------------------------------------------");

    // Display second arm's servos
    mvwprintw(win, 12, 0, "Arm 2:");
    for (size_t i = 0; i < arm::SERVO_COUNT; ++i)
    {
        int row = i + 13;
        const auto& servo = arm2_data[i];

        if (servo.error.empty())
        {
            if (servo.mirroring && has_colors())
            {
                wattron(win, COLOR_PAIR(4) | A_BOLD);
            }

            mvwprintw(win, row, 2, "%-8d %8u  %8u  %8u  [Torque: %5d] ",
                      static_cast<int>(i + 1),
                      servo.current,
                      servo.min,
                      servo.max,
                      servo.torque);

            // Display torque bar
            display_torque_bar(win, row, 42, servo.torque);

            // Display position bar with adjusted x position to make room for torque
            display_progress_bar(win, row, 49, servo.current, servo.min, servo.max);

            if (servo.mirroring && has_colors())
            {
                wattroff(win, COLOR_PAIR(4) | A_BOLD);
            }
        }
        else
        {
            mvwprintw(win, row, 2, "%-8d Error: %s",
                      static_cast<int>(i + 1),
                      servo.error.c_str());
        }
    }

    mvwprintw(win, 19, 0, "--------------------------------------------------------");

    // Add instructions and working directory
    mvwprintw(win, 20, 0, "Instructions:");
    mvwprintw(win, 21, 0, "1. Move both arms through their full range of motion");
    mvwprintw(win, 22, 0, "2. Press 's' to save calibration when done");
    mvwprintw(win, 23, 0, "3. Press keys 1-6 to toggle mirroring for each servo");
    mvwprintw(win, 24, 0, "4. Press 't' to toggle torque protection (%s)",
              torque_protection.load() ? "ON" : "OFF");
    mvwprintw(win, 25, 0, "5. Press 'l' to load calibration from file");
    mvwprintw(win, 26, 0, "6. Press Ctrl+C to exit");

    // Add color legend if colors are available
    if (has_colors())
    {
        mvwprintw(win, 27, 0, "Legend: ");
        wattron(win, COLOR_PAIR(4) | A_BOLD);
        wprintw(win, "Yellow rows = Mirrored servos");
        wattroff(win, COLOR_PAIR(4) | A_BOLD);
    }
    mvwprintw(win, 28, 0, "Save directory: %s", get_working_directory().c_str());

    wrefresh(win);
}

void export_calibration_data(const std::vector<ServoData>& arm1_data,
                             const std::vector<ServoData>& arm2_data,
                             const std::string& port1,
                             const std::string& port2)
{
    try
    {
        YAML::Node config;

        // Add metadata including timestamp
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time), "%Y-%m-%d_%H-%M-%S");
        config["timestamp"] = ss.str();

        // Add port information
        config["arm1_port"] = port1;
        config["arm2_port"] = port2;

        // Add calibration data for arm 1
        YAML::Node arm1_node;
        for (size_t i = 0; i < arm1_data.size(); ++i)
        {
            YAML::Node servo;
            servo["id"] = i + 1;
            servo["min"] = arm1_data[i].min;
            servo["max"] = arm1_data[i].max;
            // Add current mirroring state
            servo["mirroring"] = false;  // Arm 1 servos are never mirrored
            arm1_node["servos"].push_back(servo);
        }
        config["arm1"] = arm1_node;

        // Add calibration data for arm 2
        YAML::Node arm2_node;
        for (size_t i = 0; i < arm2_data.size(); ++i)
        {
            YAML::Node servo;
            servo["id"] = i + 1;
            servo["min"] = arm2_data[i].min;
            servo["max"] = arm2_data[i].max;
            // Include mirroring state for arm 2
            servo["mirroring"] = arm2_data[i].mirroring;
            arm2_node["servos"].push_back(servo);
        }
        config["arm2"] = arm2_node;

        // Fixed filename for consistency
        const std::string filename = "arm_calibration.yaml";

        // Open file with overwrite permissions
        std::ofstream fout(filename, std::ios::out | std::ios::trunc);
        if (!fout.is_open())
        {
            throw std::runtime_error("Failed to open file for writing: " + filename);
        }

        // Write configuration with error checking
        fout << config;
        if (fout.fail())
        {
            fout.close();
            throw std::runtime_error("Failed to write data to file: " + filename);
        }

        // Ensure all data is written and close file
        fout.flush();
        if (fout.fail())
        {
            fout.close();
            throw std::runtime_error("Failed to flush data to file: " + filename);
        }
        fout.close();

        // Check for any errors that occurred during close
        if (fout.fail())
        {
            throw std::runtime_error("Error occurred while closing file: " + filename);
        }
    }
    catch (const YAML::Exception& e)
    {
        throw std::runtime_error(std::string("YAML error while saving calibration: ") + e.what());
    }
    catch (const std::exception& e)
    {
        throw;  // Re-throw other exceptions
    }
}

void load_calibration_data(std::vector<ServoData>& arm1_data,
                           std::vector<ServoData>& arm2_data,
                           const std::string& filename = "arm_calibration.yaml")
{
    try
    {
        YAML::Node config = YAML::LoadFile(filename);

        // Load arm 1 servo data
        if (config["arm1"] && config["arm1"]["servos"])
        {
            auto arm1_servos = config["arm1"]["servos"];
            for (size_t i = 0; i < std::min(arm1_data.size(), arm1_servos.size()); ++i)
            {
                auto servo = arm1_servos[i];
                if (servo["min"] && servo["max"])
                {
                    arm1_data[i].min = servo["min"].as<uint16_t>();
                    arm1_data[i].max = servo["max"].as<uint16_t>();
                    // Ensure current position stays within new bounds
                    arm1_data[i].current = std::max(arm1_data[i].min,
                                                    std::min(arm1_data[i].max,
                                                             arm1_data[i].current));
                }
            }
        }

        // Load arm 2 servo data
        if (config["arm2"] && config["arm2"]["servos"])
        {
            auto arm2_servos = config["arm2"]["servos"];
            for (size_t i = 0; i < std::min(arm2_data.size(), arm2_servos.size()); ++i)
            {
                auto servo = arm2_servos[i];
                if (servo["min"] && servo["max"])
                {
                    arm2_data[i].min = servo["min"].as<uint16_t>();
                    arm2_data[i].max = servo["max"].as<uint16_t>();
                    // Ensure current position stays within new bounds
                    arm2_data[i].current = std::max(arm2_data[i].min,
                                                    std::min(arm2_data[i].max,
                                                             arm2_data[i].current));
                }
                if (servo["mirroring"])
                {
                    arm2_data[i].mirroring = servo["mirroring"].as<bool>();
                }
            }
        }
    }
    catch (const YAML::Exception& e)
    {
        throw std::runtime_error(std::string("YAML error while loading calibration: ") + e.what());
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error(std::string("Error loading calibration: ") + e.what());
    }
}

// Disable torque on every servo of both arms, then tear down ncurses. Must
// only be called from normal (non-signal) context while the readers are alive.
void disable_torque_and_cleanup(ST3215ServoReader* reader1, ST3215ServoReader* reader2)
{
    if (ncurses_win != nullptr)
    {
        mvwprintw(ncurses_win, ui::STATUS_ROW, 0, "Disabling servo torque...");
        wrefresh(ncurses_win);
    }

    for (ST3215ServoReader* reader : {reader1, reader2})
    {
        if (reader == nullptr)
        {
            continue;
        }
        for (uint8_t id = arm::FIRST_SERVO_ID; id < arm::FIRST_SERVO_ID + arm::SERVO_COUNT; ++id)
        {
            try
            {
                reader->write_control_register(id, register_addr::TORQUE_ENABLE, torque_switch::OFF);
            }
            catch (...)
            {
                // Ignore errors during shutdown
            }
        }
    }

    // Clean up ncurses
    if (ncurses_win != nullptr)
    {
        endwin();
        ncurses_win = nullptr;
    }
}

// Async-signal-safe: only stores to a lock-free atomic.
extern "C" void handle_shutdown_signal(int /*signum*/)
{
    running.store(false);
}

void install_signal_handlers()
{
    struct sigaction action{};
    action.sa_handler = handle_shutdown_signal;
    sigemptyset(&action.sa_mask);
    // No SA_RESTART: blocking reads (the std::cin port prompts) return EINTR
    // so start-up can be aborted with Ctrl+C.
    action.sa_flags = 0;
    for (const int signum : {SIGINT, SIGTERM, SIGHUP})
    {
        if (sigaction(signum, &action, nullptr) != 0)
        {
            throw std::runtime_error("Failed to install signal handler");
        }
    }
}

// Print a status line, hold it on screen for `duration`, then clear it.
void flash_status(int row, std::chrono::milliseconds duration, int color_pair, const char* format, ...)
    __attribute__((format(printf, 4, 5)));

void flash_status(int row, std::chrono::milliseconds duration, int color_pair, const char* format, ...)
{
    wmove(ncurses_win, row, 0);
    wclrtoeol(ncurses_win);
    if (color_pair != 0 && has_colors())
    {
        wattron(ncurses_win, COLOR_PAIR(color_pair) | A_BOLD);
    }
    va_list args;
    va_start(args, format);
    vw_printw(ncurses_win, format, args);
    va_end(args);
    if (color_pair != 0 && has_colors())
    {
        wattroff(ncurses_win, COLOR_PAIR(color_pair) | A_BOLD);
    }
    wrefresh(ncurses_win);
    std::this_thread::sleep_for(duration);
    wmove(ncurses_win, row, 0);
    wclrtoeol(ncurses_win);
    wrefresh(ncurses_win);
}

// Block until a key is pressed or a shutdown signal arrives.
void wait_for_key()
{
    wtimeout(ncurses_win, ui::KEY_WAIT_POLL_MS);
    while (running.load() && wgetch(ncurses_win) == ERR)
    {
    }
    nodelay(ncurses_win, TRUE);  // Back to non-blocking for the main loop
}

struct Options
{
    std::string port1;
    std::string port2;
    unsigned int baud_rate = communication::DEFAULT_BAUD_RATE;
};

void print_usage(const char* program)
{
    std::cout << "Usage: " << program << " [--baud <rate>] [<arm1-port> <arm2-port>]\n"
              << "  --baud <rate>  Serial baud rate for both arms (default "
              << communication::DEFAULT_BAUD_RATE << ", the ST3215 factory default)\n"
              << "  Without ports, available serial ports are listed for selection.\n";
}

// Returns false if the program should exit (eg --help).
bool parse_options(int argc, char* argv[], Options& options)
{
    std::vector<std::string> positional;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        std::string baud_text;
        if (arg == "-h" || arg == "--help")
        {
            print_usage(argv[0]);
            return false;
        }
        if (arg == "--baud" || arg == "-b")
        {
            if (i + 1 >= argc)
            {
                throw std::invalid_argument("--baud requires a value");
            }
            baud_text = argv[++i];
        }
        else if (arg.rfind("--baud=", 0) == 0)
        {
            baud_text = arg.substr(std::string("--baud=").size());
        }
        else if (!arg.empty() && arg[0] == '-')
        {
            throw std::invalid_argument("Unknown option: " + arg);
        }
        else
        {
            positional.push_back(arg);
            continue;
        }

        std::size_t consumed = 0;
        unsigned long baud = 0;
        try
        {
            baud = std::stoul(baud_text, &consumed);
        }
        catch (const std::exception&)
        {
            consumed = 0;
        }
        if (consumed != baud_text.size() || baud == 0 || baud > std::numeric_limits<unsigned int>::max())
        {
            throw std::invalid_argument("Invalid baud rate: " + baud_text);
        }
        options.baud_rate = static_cast<unsigned int>(baud);
    }

    if (positional.size() == 2)
    {
        options.port1 = positional[0];
        options.port2 = positional[1];
    }
    else if (!positional.empty())
    {
        throw std::invalid_argument("Expected either no ports or exactly two (arm 1 and arm 2)");
    }
    return true;
}

int main(int argc, char* argv[])
{
    std::vector<ServoData> arm1_data(arm::SERVO_COUNT);
    std::vector<ServoData> arm2_data(arm::SERVO_COUNT);

    // Declared outside the try so the error path can still disable torque
    // through live readers (they used to be destroyed before the catch ran).
    std::unique_ptr<ST3215ServoReader> reader1;
    std::unique_ptr<ST3215ServoReader> reader2;

    try
    {
        Options options;
        try
        {
            if (!parse_options(argc, argv, options))
            {
                return 0;
            }
        }
        catch (const std::invalid_argument& e)
        {
            std::cerr << "Error: " << e.what() << "\n";
            print_usage(argv[0]);
            return 2;
        }

        // Install before initscr() so ncurses does not install its own
        // SIGINT/SIGTERM handlers (which would exit without torque-off).
        install_signal_handlers();

        // Get port paths
        std::string port_path1 = options.port1;
        std::string port_path2 = options.port2;
        if (port_path1.empty())
        {
            auto available_ports = find_serial_ports();
            auto [p1, p2] = select_serial_ports(available_ports);
            port_path1 = p1;
            port_path2 = p2;
        }

        std::cout << "Using serial ports:\nArm 1: " << port_path1
                  << "\nArm 2: " << port_path2
                  << "\nBaud rate: " << options.baud_rate << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (!running.load())
        {
            throw ShutdownRequested();
        }

        // Initialize ncurses
        ncurses_win = initscr();
        cbreak();
        noecho();
        curs_set(0);
        nodelay(ncurses_win, TRUE);
        keypad(ncurses_win, TRUE);

        // Initialize servo readers for both arms
        reader1 = std::make_unique<ST3215ServoReader>(port_path1, options.baud_rate,
                                                      communication::DEFAULT_ACCELERATION);
        reader2 = std::make_unique<ST3215ServoReader>(port_path2, options.baud_rate,
                                                      communication::DEFAULT_ACCELERATION);

        // Initialize colors if terminal supports them
        if (has_colors())
        {
            start_color();
            init_pair(1, COLOR_BLUE, COLOR_BLACK);    // For min value
            init_pair(2, COLOR_GREEN, COLOR_BLACK);   // For max value
            init_pair(3, COLOR_WHITE, COLOR_BLACK);   // For current value
            init_pair(4, COLOR_YELLOW, COLOR_BLACK);  // For mirrored servos / medium torque
            init_pair(5, COLOR_RED, COLOR_BLACK);     // For high torque
            init_pair(6, COLOR_WHITE, COLOR_BLACK);   // For low torque (grey)
        }

        // Main loop
        while (running.load())
        {
            // Read all servo positions and torque from first arm
            for (size_t i = 0; i < arm::SERVO_COUNT && running.load(); ++i)
            {
                const auto id = static_cast<uint8_t>(arm::FIRST_SERVO_ID + i);
                try
                {
                    auto& servo = arm1_data[i];
                    servo.current = reader1->read_position(id);
                    servo.min = std::min(servo.min, servo.current);
                    servo.max = std::max(servo.max, servo.current);

                    // Read torque value
                    int16_t torque = reader1->read_load(id);
                    servo.torque = torque;

                    // Check if torque exceeds safety threshold
                    if (torque_protection.load() && std::abs(torque) > limits::TORQUE_SAFETY_THRESHOLD)
                    {
                        // Disable torque
                        reader1->write_control_register(id, register_addr::TORQUE_ENABLE, torque_switch::OFF);
                        servo.error = "Torque limit exceeded - disabled";
                    }
                    else if (!servo.error.empty())
                    {
                        servo.error.clear();
                    }
                }
                catch (const std::exception& e)
                {
                    arm1_data[i].error = e.what();
                    arm1_data[i].torque = 0;  // Clear torque on error
                }
            }

            // Read all servo positions and torque from second arm
            for (size_t i = 0; i < arm::SERVO_COUNT && running.load(); ++i)
            {
                const auto id = static_cast<uint8_t>(arm::FIRST_SERVO_ID + i);
                try
                {
                    auto& servo = arm2_data[i];
                    servo.current = reader2->read_position(id);
                    servo.min = std::min(servo.min, servo.current);
                    servo.max = std::max(servo.max, servo.current);

                    // Read torque value if servo is enabled (mirroring)
                    if (servo.mirroring)
                    {
                        int16_t torque = reader2->read_load(id);
                        servo.torque = torque;

                        // Only apply software torque protection if enabled
                        if (torque_protection.load() && std::abs(torque) > limits::TORQUE_SAFETY_THRESHOLD)
                        {
                            // Disable torque
                            reader2->write_control_register(id, register_addr::TORQUE_ENABLE, torque_switch::OFF);
                            servo.error = "Torque limit exceeded - disabled";
                            servo.mirroring = false;  // Disable mirroring when torque limit is exceeded
                        }
                        else if (servo.error.find("Overload") != std::string::npos)
                        {
                            // If we have an overload error but torque protection is off, try to re-enable
                            try
                            {
                                reader2->write_control_register(id, register_addr::TORQUE_ENABLE, torque_switch::ON);
                                servo.error.clear();
                            }
                            catch (const std::exception& e)
                            {
                                // If re-enable fails, keep the error
                                servo.error = std::string("Failed to re-enable servo: ") + e.what();
                            }
                        }
                        else
                        {
                            servo.error.clear();  // Clear any previous errors
                        }
                    }
                    else
                    {
                        servo.torque = 0;  // Zero torque when servo is not mirroring
                    }

                    if (!servo.error.empty() && servo.mirroring)
                    {
                        servo.error.clear();
                    }
                }
                catch (const std::exception& e)
                {
                    arm2_data[i].error = e.what();
                    arm2_data[i].torque = 0;  // Clear torque on error
                    if (arm2_data[i].mirroring)
                    {
                        arm2_data[i].mirroring = false;  // Disable mirroring on error
                    }
                }
            }

            if (!running.load())
            {
                break;
            }

            // Update display with both arms' data
            display_servo_values(ncurses_win, arm1_data, arm2_data);

            // Handle keyboard input
            const int ch = wgetch(ncurses_win);
            if (ch == 't' || ch == 'T')
            {
                const bool new_state = !torque_protection.load();
                torque_protection.store(new_state);
                flash_status(ui::STATUS_ROW, ui::STATUS_MESSAGE_TIME, new_state ? 2 : 5,
                             "Torque protection %s", new_state ? "ENABLED" : "DISABLED");
            }
            else if (ch == 'l' || ch == 'L')
            {
                mvwprintw(ncurses_win, ui::LOAD_STATUS_ROW, 0, "Loading calibration data...");
                wrefresh(ncurses_win);

                try
                {
                    load_calibration_data(arm1_data, arm2_data);
                    flash_status(ui::LOAD_STATUS_ROW, ui::STATUS_MESSAGE_TIME, 0,
                                 "Calibration data loaded successfully!");
                }
                catch (const std::exception& e)
                {
                    flash_status(ui::LOAD_STATUS_ROW, ui::ERROR_MESSAGE_TIME, 0,
                                 "Error loading calibration: %s", e.what());
                }

                // Apply the loaded mirroring states to arm 2's torque
                for (size_t i = 0; i < arm::SERVO_COUNT; ++i)
                {
                    try
                    {
                        reader2->write_control_register(static_cast<uint8_t>(arm::FIRST_SERVO_ID + i),
                                                        register_addr::TORQUE_ENABLE,
                                                        arm2_data[i].mirroring ? torque_switch::ON
                                                                               : torque_switch::OFF);
                    }
                    catch (const std::exception& e)
                    {
                        arm2_data[i].error = std::string("Failed to set torque: ") + e.what();
                        arm2_data[i].mirroring = false;
                    }
                }
            }
            else if (ch == 's' || ch == 'S')
            {
                mvwprintw(ncurses_win, ui::SAVE_STATUS_ROW, 0, "Saving calibration data...");
                wrefresh(ncurses_win);

                try
                {
                    export_calibration_data(arm1_data, arm2_data, port_path1, port_path2);
                    wmove(ncurses_win, ui::SAVE_STATUS_ROW, 0);
                    wclrtoeol(ncurses_win);
                    mvwprintw(ncurses_win, ui::SAVE_STATUS_ROW, 0,
                              "Calibration data saved successfully! Press any key to continue");
                    wrefresh(ncurses_win);
                    wait_for_key();
                    wmove(ncurses_win, ui::SAVE_STATUS_ROW, 0);
                    wclrtoeol(ncurses_win);
                    wrefresh(ncurses_win);
                }
                catch (const std::exception& e)
                {
                    flash_status(ui::SAVE_STATUS_ROW, ui::ERROR_MESSAGE_TIME, 0,
                                 "Error saving calibration: %s", e.what());
                }
            }
            else if (ch >= '1' && ch < '1' + static_cast<int>(arm::SERVO_COUNT))
            {
                const auto servo_idx = static_cast<size_t>(ch - '1');
                const auto id = static_cast<uint8_t>(arm::FIRST_SERVO_ID + servo_idx);
                const bool new_mirror_state = !arm2_data[servo_idx].mirroring;
                arm2_data[servo_idx].mirroring = new_mirror_state;

                try
                {
                    // Follower torque is on exactly while the servo is mirroring
                    reader2->write_control_register(id, register_addr::TORQUE_ENABLE,
                                                    new_mirror_state ? torque_switch::ON : torque_switch::OFF);
                    flash_status(ui::STATUS_ROW, ui::STATUS_MESSAGE_TIME, 4,
                                 "Servo %d mirroring %s, torque %s", static_cast<int>(id),
                                 new_mirror_state ? "enabled" : "disabled",
                                 new_mirror_state ? "enabled" : "disabled");
                }
                catch (const std::exception& e)
                {
                    arm2_data[servo_idx].mirroring = !new_mirror_state;  // Revert mirroring state on error
                    flash_status(ui::STATUS_ROW, ui::STATUS_MESSAGE_TIME, 0,
                                 "Error controlling torque for servo %d: %s", static_cast<int>(id), e.what());
                }
            }

            // Leader-follower: drive each mirrored arm 2 servo to arm 1's
            // position, scaled from arm 1's calibrated range to arm 2's.
            for (size_t i = 0; i < arm::SERVO_COUNT && running.load(); ++i)
            {
                if (arm2_data[i].mirroring && arm1_data[i].error.empty() && arm2_data[i].error.empty())
                {
                    try
                    {
                        const uint16_t scaled_pos = scale_position(
                            arm1_data[i].current,
                            arm1_data[i].min,
                            arm1_data[i].max,
                            arm2_data[i].min,
                            arm2_data[i].max);

                        reader2->write_position(static_cast<uint8_t>(arm::FIRST_SERVO_ID + i), scaled_pos);

                        // Update the current position in our data structure
                        arm2_data[i].current = scaled_pos;
                    }
                    catch (const std::exception& e)
                    {
                        arm2_data[i].error = std::string("Mirror error: ") + e.what();
                        arm2_data[i].mirroring = false;  // Disable mirroring on error
                        arm2_data[i].torque = 0;         // Clear torque when mirroring fails
                    }
                }
            }

            // Delay to prevent overwhelming servos
            std::this_thread::sleep_for(std::chrono::milliseconds(ui::SERVO_REFRESH_DELAY_MS));
        }

        // Shutdown requested (signal): torque off while the readers are alive
        disable_torque_and_cleanup(reader1.get(), reader2.get());
        std::cout << "Program terminated by user." << std::endl;
        return 0;
    }
    catch (const ShutdownRequested&)
    {
        disable_torque_and_cleanup(reader1.get(), reader2.get());
        std::cout << "\nProgram terminated by user." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        disable_torque_and_cleanup(reader1.get(), reader2.get());
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
