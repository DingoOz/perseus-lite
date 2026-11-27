#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "perseus_terrain/terrain_mapper.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<perseus_terrain::TerrainMapper>());
    rclcpp::shutdown();
    return 0;
}
