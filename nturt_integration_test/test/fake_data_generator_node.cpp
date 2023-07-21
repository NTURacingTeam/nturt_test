// glibc include
#include <stdint.h>

// std include
#include <chrono>
#include <functional>
#include <memory>

// ros2 include
#include <rclcpp/rclcpp.hpp>

// ros2 message include
#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// nturt include
#include "nturt_can_config.h"
#include "nturt_can_config/can_callback_register.hpp"
#include "nturt_can_config_bms-binutil.h"
#include "nturt_can_config_front_sensor-binutil.h"
#include "nturt_can_config_imu-binutil.h"
#include "nturt_can_config_inv-binutil.h"
#include "nturt_can_config_rear_sensor-binutil.h"
#include "nturt_can_config_vcu-binutil.h"

using namespace std::chrono_literals;

/* mcaro ---------------------------------------------------------------------*/
/**
 * @brief Macro for incrementing data in [MIN, MAX] with INCREMENT.
 *
 * @param DATA Data to be incremented.
 * @param INCREMENT Increment value.
 * @param MIN Minimum value.
 * @param MAX Maximum value.
 *
 */
#define INCREMENT_DATA(DATA, INCREMENT, MIN, MAX) \
  do {                                            \
    if (DATA < MIN) {                             \
      DATA = MIN;                                 \
    }                                             \
    DATA += INCREMENT;                            \
    if (DATA > MAX) {                             \
      DATA = MIN;                                 \
    }                                             \
  } while (0)

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for generating fake data for testing purposes.
 */
class FakeDataGenerator : public rclcpp::Node {
 public:
  /// @brief Constructor of FakeDataGenerator.
  FakeDataGenerator(rclcpp::NodeOptions options)
      : Node("increasing_data_test_node", options),
        can_pub_(
            this->create_publisher<can_msgs::msg::Frame>("/from_can_bus", 50)),
        gps_fix_pub_(
            this->create_publisher<sensor_msgs::msg::NavSatFix>("/fix", 10)),
        gps_vel_pub_(this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/vel", 10)),
        fast_update_data_timer_(this->create_wall_timer(
            10ms, std::bind(&FakeDataGenerator::fast_update_data_timer_callback,
                            this))),
        slow_update_data_timer_(this->create_wall_timer(
            1s, std::bind(&FakeDataGenerator::slow_update_data_timer_callback,
                          this))),
        send_data_timer_(this->create_wall_timer(
            10ms,
            std::bind(&FakeDataGenerator::send_data_timer_callback, this))) {
    // init can frame data
    memset(&bms_tx_, 0, sizeof(bms_tx_));
    memset(&front_sensor_tx_, 0, sizeof(front_sensor_tx_));
    memset(&imu_tx_, 0, sizeof(imu_tx_));
    memset(&inv_tx_, 0, sizeof(inv_tx_));
    memset(&rear_sensor_tx_, 0, sizeof(rear_sensor_tx_));
    memset(&vcu_tx_, 0, sizeof(vcu_tx_));
  };

  void register_callback() {
    CanCallbackRegieter::register_callback(
        static_cast<get_tick_t>(std::bind(&FakeDataGenerator::get_tick, this)));
    CanCallbackRegieter::register_callback(static_cast<send_can_message_t>(
        std::bind(&FakeDataGenerator::send_can_message, this,
                  std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4)));
  }

 private:
  /// @brief Timed callback function for for fast updating data.
  void fast_update_data_timer_callback() {
    // front_sensor_1
    FRONT_SENSOR_1_t* front_sensor_1 = &front_sensor_tx_.FRONT_SENSOR_1;
    INCREMENT_DATA(front_sensor_1->FRONT_SENSOR_Brake_phys, 0.01, 0, 1);
    INCREMENT_DATA(front_sensor_1->FRONT_SENSOR_Accelerator_1_phys, 0.01, 0, 1);
    INCREMENT_DATA(front_sensor_1->FRONT_SENSOR_Accelerator_2_phys, 0.01, 0, 1);
    INCREMENT_DATA(front_sensor_1->FRONT_SENSOR_Steer_Angle, 1, -90, 90);

    // front_sensor_2
    FRONT_SENSOR_2_t* front_sensor_2 = &front_sensor_tx_.FRONT_SENSOR_2;
    INCREMENT_DATA(front_sensor_2->FRONT_SENSOR_Front_Left_Wheel_Speed_phys, 10,
                   0, 600);
    INCREMENT_DATA(front_sensor_2->FRONT_SENSOR_Front_Right_Wheel_Speed_phys,
                   10, 0, 600);
    INCREMENT_DATA(front_sensor_2->FRONT_SENSOR_Front_Left_Suspension_phys,
                   0.01, 30, 50);
    INCREMENT_DATA(front_sensor_2->FRONT_SENSOR_Front_Right_Suspension_phys,
                   0.01, 30, 50);
    INCREMENT_DATA(front_sensor_2->FRONT_SENSOR_Front_Brake_Pressure_phys, 0.1,
                   0, 20);
    INCREMENT_DATA(front_sensor_2->FRONT_SENSOR_Rear_Brake_Pressure_phys, 0.1,
                   0, 20);

    // front_sensor_3
    FRONT_SENSOR_3_t* front_sensor_3 = &front_sensor_tx_.FRONT_SENSOR_3;
    INCREMENT_DATA(
        front_sensor_3->FRONT_SENSOR_Front_Left_Tire_Temperature_1_phys, 0.1,
        30, 80);
    INCREMENT_DATA(
        front_sensor_3->FRONT_SENSOR_Front_Left_Tire_Temperature_2_phys, 0.1,
        30, 80);
    INCREMENT_DATA(
        front_sensor_3->FRONT_SENSOR_Front_Left_Tire_Temperature_3_phys, 0.1,
        30, 80);
    INCREMENT_DATA(
        front_sensor_3->FRONT_SENSOR_Front_Left_Tire_Temperature_4_phys, 0.1,
        30, 80);
    INCREMENT_DATA(
        front_sensor_3->FRONT_SENSOR_Front_Right_Tire_Temperature_1_phys, 0.1,
        30, 80);
    INCREMENT_DATA(
        front_sensor_3->FRONT_SENSOR_Front_Right_Tire_Temperature_2_phys, 0.1,
        30, 80);
    INCREMENT_DATA(
        front_sensor_3->FRONT_SENSOR_Front_Right_Tire_Temperature_3_phys, 0.1,
        30, 80);
    INCREMENT_DATA(
        front_sensor_3->FRONT_SENSOR_Front_Right_Tire_Temperature_4_phys, 0.1,
        30, 80);

    // rear_sensor_1
    REAR_SENSOR_1_t* rear_sensor_1 = &rear_sensor_tx_.REAR_SENSOR_1;
    INCREMENT_DATA(rear_sensor_1->REAR_SENSOR_Rear_Left_Wheel_Speed_phys, 10, 0,
                   600);
    INCREMENT_DATA(rear_sensor_1->REAR_SENSOR_Rear_Right_Wheel_Speed_phys, 10,
                   0, 600);
    INCREMENT_DATA(rear_sensor_1->REAR_SENSOR_Rear_Left_Suspension_phys, 0.01,
                   30, 50);
    INCREMENT_DATA(rear_sensor_1->REAR_SENSOR_Rear_Right_Suspension_phys, 0.01,
                   30, 50);

    // rear_sensor_2
    REAR_SENSOR_2_t* rear_sensor_2 = &rear_sensor_tx_.REAR_SENSOR_2;
    INCREMENT_DATA(rear_sensor_2->REAR_SENSOR_Rear_Left_Tire_Temperature_1_phys,
                   0.1, 30, 80);
    INCREMENT_DATA(rear_sensor_2->REAR_SENSOR_Rear_Left_Tire_Temperature_2_phys,
                   0.1, 30, 80);
    INCREMENT_DATA(rear_sensor_2->REAR_SENSOR_Rear_Left_Tire_Temperature_3_phys,
                   0.1, 30, 80);
    INCREMENT_DATA(rear_sensor_2->REAR_SENSOR_Rear_Left_Tire_Temperature_4_phys,
                   0.1, 30, 80);
    INCREMENT_DATA(
        rear_sensor_2->REAR_SENSOR_Rear_Right_Tire_Temperature_1_phys, 0.1, 30,
        80);
    INCREMENT_DATA(
        rear_sensor_2->REAR_SENSOR_Rear_Right_Tire_Temperature_2_phys, 0.1, 30,
        80);
    INCREMENT_DATA(
        rear_sensor_2->REAR_SENSOR_Rear_Right_Tire_Temperature_3_phys, 0.1, 30,
        80);
    INCREMENT_DATA(
        rear_sensor_2->REAR_SENSOR_Rear_Right_Tire_Temperature_4_phys, 0.1, 30,
        80);

    // bms_cell_stats
    BMS_Cell_Stats_t* bms_cell_stats = &bms_tx_.BMS_Cell_Stats;
    INCREMENT_DATA(bms_cell_stats->BMS_Segment_Index, 1, 0, 6);
    INCREMENT_DATA(bms_cell_stats->BMS_Cell_Index, 1, 0, 3);
    INCREMENT_DATA(bms_cell_stats->BMS_Cell_Voltage_1_phys, 0.01, 2.8, 4.2);
    INCREMENT_DATA(bms_cell_stats->BMS_Cell_Voltage_2_phys, 0.01, 2.8, 4.2);
    INCREMENT_DATA(bms_cell_stats->BMS_Cell_Voltage_3_phys, 0.01, 2.8, 4.2);
    INCREMENT_DATA(bms_cell_stats->BMS_Cell_Temperature_1_phys, 1, 0, 100);
    INCREMENT_DATA(bms_cell_stats->BMS_Cell_Temperature_2_phys, 1, 0, 100);
    INCREMENT_DATA(bms_cell_stats->BMS_Cell_Temperature_3_phys, 1, 0, 100);

    // inverter_fast_info
    INV_Fast_Info_t* inverter_fast_info = &inv_tx_.INV_Fast_Info;
    INCREMENT_DATA(inverter_fast_info->INV_Fast_Torque_Command_phys, 1, 0, 100);
    INCREMENT_DATA(inverter_fast_info->INV_Fast_Torque_Feedback_phys, 1, 0,
                   100);
    INCREMENT_DATA(inverter_fast_info->INV_Fast_Motor_Speed, 10, 0, 5000);
    INCREMENT_DATA(inverter_fast_info->INV_Fast_DC_Bus_Voltage_phys, 5, 0, 500);

    // inverter other
    INCREMENT_DATA(inv_tx_.INV_Temperature_Set_2.INV_Control_Board_Temp_phys, 1,
                   0, 100);
    INCREMENT_DATA(inv_tx_.INV_Temperature_Set_3.INV_Hot_Spot_Temp_phys, 1, 0,
                   100);
    INCREMENT_DATA(inv_tx_.INV_Temperature_Set_3.INV_Motor_Temp_phys, 1, 0,
                   100);
    INCREMENT_DATA(inv_tx_.INV_Current_Info.INV_DC_Bus_Current_phys, 2, 0, 200);

    // imu_acceleration
    IMU_Acceleration_t* imu_acceleration = &imu_tx_.IMU_Acceleration;
    INCREMENT_DATA(imu_acceleration->IMU_Acceleration_X_phys, 0.01, -2, 2);
    INCREMENT_DATA(imu_acceleration->IMU_Acceleration_Y_phys, 0.01, -2, 2);
    INCREMENT_DATA(imu_acceleration->IMU_Acceleration_Z_phys, 0.01, -2, 2);

    // imu_angular_velocity
    IMU_Angular_Velocity_t* imu_angular_velocity =
        &imu_tx_.IMU_Angular_Velocity;
    INCREMENT_DATA(imu_angular_velocity->IMU_Angular_Velocity_X_phys, 10, -360,
                   360);
    INCREMENT_DATA(imu_angular_velocity->IMU_Angular_Velocity_Y_phys, 10, -360,
                   360);
    INCREMENT_DATA(imu_angular_velocity->IMU_Angular_Velocity_Z_phys, 10, -360,
                   360);

    // imu_quaternion
    IMU_Quaternion_t* imu_quaternion = &imu_tx_.IMU_Quaternion;
    INCREMENT_DATA(imu_quaternion->IMU_Quaternion_W_phys, 0.01, -1, 1);
    INCREMENT_DATA(imu_quaternion->IMU_Quaternion_X_phys, 0.01, -1, 1);
    INCREMENT_DATA(imu_quaternion->IMU_Quaternion_Y_phys, 0.01, -1, 1);
    INCREMENT_DATA(imu_quaternion->IMU_Quaternion_Z_phys, 0.01, -1, 1);
  }

  /// @brief Timed callback function for for slow updating data.
  void slow_update_data_timer_callback() {
    // vcu_status
    VCU_Status_t* vcu_status = &vcu_tx_.VCU_Status;
    INCREMENT_DATA(vcu_status->VCU_Status, 1, 0, 4);
    INCREMENT_DATA(vcu_status->VCU_Error_Code, 4, 0, 4);

    // front_sensor_1
    FRONT_SENSOR_1_t* front_sensor_1 = &front_sensor_tx_.FRONT_SENSOR_1;
    INCREMENT_DATA(front_sensor_1->FRONT_SENSOR_Brake_Micro, 1, 0, 1);
    INCREMENT_DATA(front_sensor_1->FRONT_SENSOR_Accelerator_Micro, 1, 0, 1);

    // rear_sensor_status
    REAR_SENSOR_Status_t* rear_sensor_status =
        &rear_sensor_tx_.REAR_SENSOR_Status;
    INCREMENT_DATA(rear_sensor_status->REAR_SENSOR_Status, 4, 0, 4);
    INCREMENT_DATA(rear_sensor_status->REAR_SENSOR_Error_Code, 1, 0, 1);

    // bms_status
    BMS_Status_t* bms_status = &bms_tx_.BMS_Status;
    INCREMENT_DATA(bms_status->BMS_Error_Code, 1, 0, 1);

    // inverter_fault_codes
    INV_Fault_Codes_t* inverter_fault_codes = &inv_tx_.INV_Fault_Codes;
    INCREMENT_DATA(inverter_fault_codes->INV_Post_Fault_Lo, 20, 0, 20);
    INCREMENT_DATA(inverter_fault_codes->INV_Run_Fault_Lo, 12, 0, 12);

    // gps_fix
    gps_fix_.header.stamp = now();
    INCREMENT_DATA(gps_fix_.latitude, 0.1, 24.0, 25.0);
    INCREMENT_DATA(gps_fix_.longitude, 0.1, 121.0, 122.0);
    INCREMENT_DATA(gps_fix_.altitude, 1, 0, 100);
    gps_fix_pub_->publish(gps_fix_);

    // gps_vel
    gps_vel_.header.stamp = now();
    INCREMENT_DATA(gps_vel_.twist.linear.x, 1, 0, 100);
    INCREMENT_DATA(gps_vel_.twist.linear.y, 1, 0, 100);
    gps_vel_pub_->publish(gps_vel_);
  }

  /// @brief Timed callback function for sending data to control tower.
  void send_data_timer_callback() {
    nturt_can_config_bms_Transmit(&bms_tx_);
    nturt_can_config_front_sensor_Transmit(&front_sensor_tx_);
    nturt_can_config_imu_Transmit(&imu_tx_);
    nturt_can_config_inv_Transmit(&inv_tx_);
    nturt_can_config_rear_sensor_Transmit(&rear_sensor_tx_);
    nturt_can_config_vcu_Transmit(&vcu_tx_);
  }

  uint32_t get_tick() {
    return static_cast<uint32_t>(now().nanoseconds() / 1000000);
  }

  int send_can_message(uint32_t msgid, uint8_t ide, uint8_t* d, uint8_t len) {
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

  /// @brief ROS2 publisher to "/from_can_bus", for publishing can signal.
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;

  /// @brief ROS2 publisher to "/fix", for publishing GPS signal.
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_fix_pub_;

  /// @brief ROS2 publisher to "/vel", for publishing GPS signal.
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr gps_vel_pub_;

  /// @brief ROS2 timer for fast updating data.
  rclcpp::TimerBase::SharedPtr fast_update_data_timer_;

  /// @brief ROS2 timer for slow updating data.
  rclcpp::TimerBase::SharedPtr slow_update_data_timer_;

  /// @brief ROS2 timer for publishing updated data.
  rclcpp::TimerBase::SharedPtr send_data_timer_;

  /// @brief Struct for storing front sensor transmit can frame data.
  nturt_can_config_front_sensor_tx_t front_sensor_tx_;

  /// @brief Struct for storing bms transmit can frame data.
  nturt_can_config_bms_tx_t bms_tx_;

  /// @brief Struct for storing imu transmit can frame data.
  nturt_can_config_imu_tx_t imu_tx_;

  /// @brief Struct for storing inverter transmit can frame data.
  nturt_can_config_inv_tx_t inv_tx_;

  /// @brief Struct for storing rear sensor transmit can frame data.
  nturt_can_config_rear_sensor_tx_t rear_sensor_tx_;

  /// @brief Struct for storing vcu transmit can frame data.
  nturt_can_config_vcu_tx_t vcu_tx_;

  /// @brief Struct for storing "/fix" message data.
  sensor_msgs::msg::NavSatFix gps_fix_;

  /// @brief Struct for storing "/vel" message data.
  geometry_msgs::msg::TwistStamped gps_vel_;
};

/* entry function ------------------------------------------------------------*/
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  rclcpp::NodeOptions options;

  auto fake_data_generator_node = std::make_shared<FakeDataGenerator>(options);
  fake_data_generator_node->register_callback();

  executor.add_node(fake_data_generator_node);
  executor.spin();
  CanCallbackRegieter::reset();
  rclcpp::shutdown();
  return 0;
}
