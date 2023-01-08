// std include
#include <memory>

// ros2 include
#include "rclcpp/rclcpp.hpp"

// nturt include
#include "nturt_rpi_can_latency_test/rpi_can_latency_test.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RpiCanLatencyTest>());
    rclcpp::shutdown();

    return 0;
}
