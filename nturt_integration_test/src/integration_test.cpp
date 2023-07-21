#include "nturt_integration_test/integration_test.hpp"

// glibc include
#include <stdint.h>
#include <string.h>

// stl include
#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <utility>

// ros2 include
#include <rclcpp/rclcpp.hpp>

// ros2 message include
#include <can_msgs/msg/frame.hpp>

// nturt include
#include "nturt_can_config.h"
#include "nturt_can_config/can_callback_register.hpp"
#include "nturt_can_config_inv-binutil.h"
#include "nturt_can_config_logger-binutil.h"
#include "nturt_can_config_rear_sensor-binutil.h"
#include "nturt_can_config_vcu-binutil.h"
#include "nturt_integration_test/user_interface.hpp"

using namespace std::chrono_literals;

IntegrationTest::IntegrationTest(rclcpp::NodeOptions options)
    : Node("nturt_integration_test_node", options),
      can_pub_(this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 50)),
      can_sub_(this->create_subscription<can_msgs::msg::Frame>(
          "/from_can_bus", 10,
          std::bind(&IntegrationTest::onCan, this, std::placeholders::_1))),
      update_can_timer_(this->create_wall_timer(
          10ms, std::bind(&IntegrationTest::update_can_timer_callback, this))) {
  // init can frame
  memset(&inv_rx_, 0, sizeof(inv_rx_));
  memset(&rear_sensor_rx_, 0, sizeof(rear_sensor_rx_));
  memset(&logger_rx_, 0, sizeof(logger_rx_));
  memset(&vcu_rx_, 0, sizeof(vcu_rx_));

  nturt_can_config_inv_Check_Receive_Timeout_Init(&inv_rx_);
  nturt_can_config_rear_sensor_Check_Receive_Timeout_Init(&rear_sensor_rx_);
  nturt_can_config_logger_Check_Receive_Timeout_Init(&logger_rx_);
  nturt_can_config_vcu_Check_Receive_Timeout_Init(&vcu_rx_);

  ui_thread_ = std::thread(std::bind(&UserInterface::mainloop, &ui_));
}

void IntegrationTest::register_can_callback() {
  CanCallbackRegieter::register_callback(
      static_cast<get_tick_t>(std::bind(&IntegrationTest::get_tick, this)));
  CanCallbackRegieter::register_callback(static_cast<fmon_mono_t>(
      std::bind(&IntegrationTest::fmon_mono, this, std::placeholders::_1,
                std::placeholders::_2)));
  CanCallbackRegieter::register_callback(static_cast<tout_mono_t>(
      std::bind(&IntegrationTest::tout_mono, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3)));
  CanCallbackRegieter::register_callback(static_cast<send_can_message_t>(
      std::bind(&IntegrationTest::send_can_message, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3,
                std::placeholders::_4)));
}

void IntegrationTest::onCan(const std::shared_ptr<can_msgs::msg::Frame> msg) {
  nturt_can_config_inv_Receive(&inv_rx_, msg->data.data(), msg->id, msg->dlc);
  nturt_can_config_rear_sensor_Receive(&rear_sensor_rx_, msg->data.data(),
                                       msg->id, msg->dlc);
  nturt_can_config_logger_Receive(&logger_rx_, msg->data.data(), msg->id,
                                  msg->dlc);
  nturt_can_config_vcu_Receive(&vcu_rx_, msg->data.data(), msg->id, msg->dlc);
}

void IntegrationTest::update_can_timer_callback() {
  nturt_can_config_inv_Check_Receive_Timeout(&inv_rx_);
  nturt_can_config_rear_sensor_Check_Receive_Timeout(&rear_sensor_rx_);
  nturt_can_config_logger_Check_Receive_Timeout(&logger_rx_);
  nturt_can_config_vcu_Check_Receive_Timeout(&vcu_rx_);

  // nturt_can_config_inv_Transmit(&inv_tx_);
  // nturt_can_config_rear_sensor_Transmit(&rear_sensor_tx_);
  // nturt_can_config_vcu_Transmit(&vcu_tx_);
}

uint32_t IntegrationTest::get_tick() {
  return static_cast<uint32_t>(now().nanoseconds() / 1000000);
}

int IntegrationTest::send_can_message(uint32_t msgid, uint8_t ide, uint8_t* d,
                                      uint8_t len) {
  can_msgs::msg::Frame msg;
  msg.id = msgid;
  msg.is_extended = ide;
  msg.dlc = len;
  for (int i = 0; i < len; i++) {
    msg.data[i] = d[i];
  }
  can_pub_->publish(msg);

  return 0;
}

void IntegrationTest::fmon_mono(FrameMonitor_t* mon, uint32_t msgid) {
  (void)msgid;

  if (mon->cycle_error) {
    mon->cycle_error = false;
  }
}

void IntegrationTest::tout_mono(FrameMonitor_t* mon, uint32_t msgid,
                                uint32_t lastcyc) {
  (void)msgid;
  (void)lastcyc;

  if (!mon->cycle_error) {
    mon->cycle_error = true;
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(IntegrationTest)
