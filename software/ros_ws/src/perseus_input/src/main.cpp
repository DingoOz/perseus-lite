#include "generic_controller/generic_controller.hpp"

// Kept separate from generic_controller.cpp so the unit tests can link the
// node implementation without a second main().
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options{};
    try
    {
        node_options.allow_undeclared_parameters(true);
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting generic controller node");
        auto node = std::make_shared<GenericController>(node_options);
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error running generic controller: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
