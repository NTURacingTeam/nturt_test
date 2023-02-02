// std include
#include <memory>

// ros2 include
#include "rclcpp/rclcpp.hpp"

// nturt include
#include "nturt_micro_ros_latency_test_host/micro_ros_latency_test_host.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    rclcpp::executors::StaticSingleThreadedExecutor executor;
    rclcpp::NodeOptions options;

    auto micro_ros_latency_test_node = std::make_shared<MicroRosLatencyTestHost>(options);

    executor.add_node(micro_ros_latency_test_node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
