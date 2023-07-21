// std include
#include <memory>

// ros2 include
#include "rclcpp/rclcpp.hpp"

// nturt include
#include "nturt_integration_test/integration_test.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  rclcpp::NodeOptions options;

  auto integration_test_node = std::make_shared<IntegrationTest>(options);
  integration_test_node->register_can_callback();

  executor.add_node(integration_test_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
