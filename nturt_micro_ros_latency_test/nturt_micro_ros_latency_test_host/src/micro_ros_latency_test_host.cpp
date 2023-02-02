#include "nturt_micro_ros_latency_test_host/micro_ros_latency_test_host.hpp"

MicroRosLatencyTestHost::MicroRosLatencyTestHost(rclcpp::NodeOptions _options)
    : Node("nturt_micro_ros_latency_test_host_node", _options),
      test_message_pub_(
          this->create_publisher<nturt_ros_interface::msg::LatencyTestMessage>(
              "/from_host", 10)),
      test_message_sub_(this->create_subscription<
                        nturt_ros_interface::msg::LatencyTestMessage>(
          "/to_host", 10,
          std::bind(&MicroRosLatencyTestHost::onTestMessage, this,
                    std::placeholders::_1))) {}

void MicroRosLatencyTestHost::onTestMessage(
    const nturt_ros_interface::msg::LatencyTestMessage &_msg) {
  test_message_pub_->publish(_msg);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(MicroRosLatencyTestHost)
