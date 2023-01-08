/**
 * @file rpi_can_latency_test.hpp
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief ROS2 package for testing rpi can latency.
 */

#ifndef RPI_CAN_LATENCY_TEST_HPP
#define RPI_CAN_LATENCY_TEST_HPP

// std include
#include <array>
#include <chrono>
#include <ctime>
#include <fstream>
#include <functional>
#include <memory>
#include <string>

// ros2 include
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"

// ros2 message include
#include "can_msgs/msg/frame.hpp"

using namespace std::chrono_literals;

/// @brief Union for type plunning between test frame information and raw frame data.
union FrameConversion {
    /// @brief Struct containing frame information.
    struct {
        /// @brief The index of this frame.
        int frame_count_;

        /// @brief The time when this frame is sent.
        float time_stemp_;
    } information;

    /// @brief Array for holding raw frame data.
    std::array<uint8_t, 8> frame_data_;
};

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for testing rpi can latency.
 */
class RpiCanLatencyTest : public rclcpp::Node {
    public:
        /// @brief Constructor of rpi_can_latency_test.
        RpiCanLatencyTest();

    private:
        /// @brief ROS2 publisher to "/to_can_bus", for sending can signal.
        rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;

        /// @brief ROS2 sbscriber to "/from_can_bus", for receiving can signal.
        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

        /// @brief ROS2 timer for periodically testing can latency.
        rclcpp::TimerBase::SharedPtr can_latency_test_timer_;

        /// @brief ROS2 timer for stopping can latency test.
        rclcpp::TimerBase::SharedPtr stopping_timer_;

        /// @brief CSV file for logging the test data.
        std::fstream csv_file_;
        
        // internal states
        /// @brief Can id that the test can message will be sent.
        uint32_t send_id_;

        /// @brief Can id that will be received as the responding can test message.
        uint32_t receive_id_;

        /// @brief Time stamp when this program starts, used for increasing the resolution of measured lateency.
        double program_start_time_;

        /// @brief Counter counting how much frame is sent.
        int frame_count_ = 0;

        /// @brief Callback function when ros is about to shutdown.
        void onShutdown();
        
        /// @brief Callback function when receiving message from "/from_can_bus".
        void onCan(const std::shared_ptr<can_msgs::msg::Frame> _msg);

        /// @brief Timed callback function for periodically testing can latency.
        void can_latency_test_callback();

        /// @brief Timed callback function for stopping can latency test.
        void stopping_callback();
};

#endif // RPI_CAN_LATENCY_TEST_HPP