/**
 * @file integration_test.hpp
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief ROS2 package for testing the whole control system.
 */

#ifndef INTEGRATION_TEST_HPP
#define INTEGRATION_TEST_HPP

// std include
#include <memory>
#include <thread>

// ros2 include
#include <rclcpp/rclcpp.hpp>

// ros2 message include
#include <can_msgs/msg/frame.hpp>

// nturt include
#include "nturt_can_config.h"
#include "nturt_can_config_inv-binutil.h"
#include "nturt_can_config_logger-binutil.h"
#include "nturt_can_config_rear_sensor-binutil.h"
#include "nturt_can_config_vcu-binutil.h"
#include "nturt_integration_test/user_interface.hpp"

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for testing the whole control system.
 */
class IntegrationTest : public rclcpp::Node {
 public:
  /// @brief Constructor of IntegrationTest.
  IntegrationTest(rclcpp::NodeOptions options);

  /// @brief Register codedbc callback function for can signal.
  void register_can_callback();

 private:
  /// @brief Callback function when receiving message from "/from_can_bus".
  void onCan(const std::shared_ptr<can_msgs::msg::Frame> msg);

  /// @brief Timed callback function for periodically updating can to check
  /// receive timeout and transmit.
  void update_can_timer_callback();

  /* coder dbc callback function ---------------------------------------------*/
  uint32_t get_tick();

  void fmon_mono(FrameMonitor_t* mon, uint32_t msgid);

  void tout_mono(FrameMonitor_t* mon, uint32_t msgid, uint32_t lastcyc);

  int send_can_message(uint32_t msgid, uint8_t ide, uint8_t* d, uint8_t len);

  /// @brief ROS2 publisher to "/to_can_bus", for publishing can signal.
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;

  /// @brief ROS2 publisher to "/from_can_bus", for receiving can signal.
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

  /// @brief ROS2 timer for updating can.
  rclcpp::TimerBase::SharedPtr update_can_timer_;

  /// @brief Front end of integration test.
  UserInterface ui_;

  /// @brief Thread for user interface.
  std::thread ui_thread_;

  /// @brief Struct for storing inverter transmit can frame data.
  nturt_can_config_inv_tx_t inv_tx_;

  /// @brief Struct for storing inverter receive can frame data.
  nturt_can_config_inv_rx_t inv_rx_;

  /// @brief Struct for storing logger receive can frame data.
  nturt_can_config_logger_rx_t logger_rx_;

  /// @brief Struct for storing rear sensor transmit can frame data.
  nturt_can_config_rear_sensor_tx_t rear_sensor_tx_;

  /// @brief Struct for storing rear sensor receive can frame data.
  nturt_can_config_rear_sensor_rx_t rear_sensor_rx_;

  /// @brief Struct for storing vcu transmit can frame data.
  nturt_can_config_vcu_tx_t vcu_tx_;

  /// @brief Struct for storing vcu receive can frame data.
  nturt_can_config_vcu_rx_t vcu_rx_;
};

/// @brief Global variable for integration_test_node.
extern std::shared_ptr<IntegrationTest> integration_test_node_global;

#endif  // INTEGRATION_TEST_HPP
