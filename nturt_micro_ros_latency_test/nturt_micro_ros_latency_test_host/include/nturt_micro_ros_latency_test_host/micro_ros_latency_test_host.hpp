/**
 * @file micro_ros_latency_test_host.hpp
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief ROS2 package for testing micro ros latency.
 */

#ifndef MICRO_ROS_LATENCY_TEST_HOST_HPP
#define MICRO_ROS_LATENCY_TEST_HOST_HPP

// std include
#include <functional>
#include <memory>

// ros2 include
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"

// nturt include
#include "nturt_ros_interface/msg/latency_test_message.hpp"

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for testing micro ros latency.
 */
class MicroRosLatencyTestHost : public rclcpp::Node {
 public:
  /// @brief Constructor.
  MicroRosLatencyTestHost(rclcpp::NodeOptions _options);

 private:
  /// @brief ROS2 publisher to "/from_host", for sending test message.
  rclcpp::Publisher<nturt_ros_interface::msg::LatencyTestMessage>::SharedPtr
      test_message_pub_;

  /// @brief ROS2 sbscriber to "/to_host", for receiving test messsage.
  rclcpp::Subscription<nturt_ros_interface::msg::LatencyTestMessage>::SharedPtr
      test_message_sub_;

  /// @brief Callback function when receiving message from "/to_host".
  void onTestMessage(const nturt_ros_interface::msg::LatencyTestMessage &_msg);
};

#endif  // MICRO_ROS_LATENCY_TEST_HOST_HPP
