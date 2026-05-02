#include <QApplication>
#include <QGuiApplication>
#include <QScreen>
#include <atomic>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "map_screen_node.hpp"
#include "map_view_widget.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);

    auto node = std::make_shared<perseus_lite_screen::MapScreenNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::atomic<bool> running{true};
    std::thread spinner([&]()
                        {
        while (running.load() && rclcpp::ok())
        {
            executor.spin_some(std::chrono::milliseconds(50));
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        } });

    perseus_lite_screen::MapViewWidget widget(node.get());
    widget.showFullScreen();

    const int rc = app.exec();

    running.store(false);
    if (spinner.joinable())
    {
        spinner.join();
    }
    rclcpp::shutdown();
    return rc;
}
