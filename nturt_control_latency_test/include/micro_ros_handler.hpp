/**
 * @file micro_ros_handler.hpp
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Micro ros package for handling micro ros.
 */

#ifndef MICRO_ROS_HANDLER_HPP
#define MICRO_ROS_HANDLER_HPP

// parameters
#define MICRO_ROS_WIFI_TRANSPORT
#define PUBLISH_RECEIVED_CAN
// #define PUBLISH_MOTOR_SPEED
// #define RANDOM_INVERTER_SIGNAL
#define SEND_INVERTER_SIGNAL

// mode
#define MONITOR_CAN_MODE

#ifdef MONITOR_CAN_MODE
#define PUBLISH_RECEIVED_CAN
#undef RECEIVE_MCU_COMMAND_ONLY
#undef SEND_INVERTER_SIGNAL
#endif

// std include
#include <cmath>
#include <mutex>
#include <numeric>
#include <vector>

// arduino include
#include <Arduino.h>

// can include
#include <CAN.h>

// micro ros include
#include <micro_ros_platformio.h>
#include <rcl/time.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ros2 interface include
#ifdef PUBLISH_RECEIVED_CAN
#include <can_msgs/msg/frame.h>
#endif

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>

/// @brief Macro function for checking error.
#define RCCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if((temp_rc != RCL_RET_OK)) { \
        error_loop(temp_rc, __LINE__, __func__); \
    } \
}

/// @brief Macro function for checking error.
#define RCSOFTCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if((temp_rc != RCL_RET_OK)){ \
        error_once(temp_rc, __LINE__, __func__); \
    } \
}

#ifdef SEND_INVERTER_SIGNAL
/// @brief Struct for storing last time the can signal was sent.
struct LastCall {
    /// @brief Board temperature.
    int64_t board_temp_;

    /// @brief Motor temperature.
    int64_t motor_temp_;

    /// @brief Motor speed.
    int64_t motor_speed_;

    /// @brief Output voltage.
    int64_t output_volt_;

    /// @brief Function to initialize in _init_time.
    /// @param[in] _init_time Initialize to _init_time.
    void init(int64_t _init_time) {
        board_temp_ = _init_time;
        motor_temp_ = _init_time;
        motor_speed_ = _init_time;
        output_volt_ = _init_time;
    }
};
#endif

/**
 * @brief Class for handling micro ros, then create one of it for use.
 * 
 * This class is implemented all in static fasion since micro ros is based on "c" and does not support using member function as callback functions.
 * And thus the purpose of this class is to reduce the enormous amout of global variables/functions floating aroud. Compacting all 
 * variables/functions used for micro ros into one class.
 * @author QuantumSpawner jet22854111@gmail.com
 */
class MicroRosHandler {
    public:
        /// @brief Function to initialize micro_ros_handler.
        static void init();

        /// @brief Function to end micro ros.
        static void end();

        /// @brief Function to spin the node inside micro ros handler.
        static void spin_once();

    private:
        /// @brief Mutex to prevent sending/receiving can signal at the same time.
        static std::mutex can_mutex_;

        /// @brief Micro ros allocator object.
        static rcl_allocator_t allocator_;

        /// @brief Micro ros support object.
        static rclc_support_t support_;

        /// @brief Micro ros executor object.
        static rclc_executor_t executor_;

        /// @brief Micro ros node.
        static rcl_node_t node_;

        /// @brief Micro ros clock.
        static rcl_clock_t clock_;
        
        /// @brief Micro ros timer for cheching can signal input, frequency of 1000 \f$[Hz]\f$.
        static rcl_timer_t ckeck_can_timer_;

        #ifdef SEND_INVERTER_SIGNAL
        /** @brief Micro ros timer for sending inverter can signal.
         * 
         * This timer is used for sending the following can signal:
         * 1. inverter board temperature, frequency of 10 \f$[Hz]\f$.
         * 2. motor temperature, frequency of 10 \f$[Hz]\f$.
         * 3. motor speed, frequency of 100 \f$[Hz]\f$.
         * 4. output voltage, frequency of 100 \f$[Hz]\f$.
         */
        static rcl_timer_t can_timer_;
        #endif

        /// @brief Micro ros timer for updating inverter motor speed, frequency of 100 \f$[Hz]\f$.
        static rcl_timer_t update_motor_speed_timer_;

        /// @brief Micro ros timer for testing control latency, frequency of 1 \f$[Hz]\f$.
        static rcl_timer_t latency_test_timer_;
        
        #ifdef PUBLISH_RECEIVED_CAN
        /// @brief Micro ros publisher to "/received_messages" for publishing received can signal.
        static rcl_publisher_t can_pub_;
        #endif

        #ifdef PUBLISH_MOTOR_SPEED
        /// @brief Micro ros publisher to "/motor_speed" for publishing motor speed.
        static rcl_publisher_t motor_speed_pub_;
        #endif

        /// @brief Micro ros publisher to "/latency" for publishing control latency in \f$[s]\f$.
        static rcl_publisher_t latency_pub_;
        
        /// @brief Micro ros subscriber to "/test_control_latency" for begin testing control latency.
        static rcl_subscription_t test_latency_sub_;

        #ifdef PUBLISH_RECEIVED_CAN
        /// @brief Micro ros message "can_msgs/Frame".
        static can_msgs__msg__Frame can_msg_;
        #endif

        #ifdef PUBLISH_MOTOR_SPEED
        /// @brief Micro ros message "std_msgs/Float32"
        static std_msgs__msg__Float32 motor_speed_msg_;
        #endif

        /// @brief Micro ros message "std_msgs/Bool"
        static std_msgs__msg__Bool test_latency_msg_;

        /// @brief Micro ros message "std_msgs/Float32"
        static std_msgs__msg__Float32 latency_msg_;

        // internal states
        #ifdef SEND_INVERTER_SIGNAL
        /// @brief Last time the can signal was sent.
        static LastCall last_call_;
        #endif

        /// @brief Torque command from can signal, volatile to cause the cpu to read from the memory everytime (insted of maybe from registers).
        /// Normally this is unsafe to share same data across multithread, but in this actomic situation, it is. Please checkout:
        /// <a href="https://stackoverflow.com/questions/3580291/what-happens-if-two-threads-read-write-the-same-piece-of-memory">stackoverflow</a>.
        static volatile float torque_command_;

        /// @brief Fake motor speed.
        static volatile float motor_speed_;

        /// @brief Flag to determined if latency test has began.
        static volatile bool begin_latency_test_;

        /** 
         * @brief Infinite loop to enter when unsolvable error happends, blink the built-in led in "sos" morse code.
         * @param _rc Error code.
         * @param _line Number of line.
         * @param _fun Name of function.
         */
        static void error_loop(const rcl_ret_t _rc, const int _line, const char* _fun);

        /** 
         * @brief Function to enter when some minor error happends, blink the built-in led once.
         * @param _rc Error code.
         * @param _line Number of line.
         * @param _fun Name of function.
         */
        static void error_once(const rcl_ret_t _rc, const int _line, const char* _fun);

        /// @brief Timed callback function to check if received inverter command can signal.
        static void check_can_callback(rcl_timer_t *_timer, int64_t _last_call_time);
        
        #ifdef SEND_INVERTER_SIGNAL
        /// @brief Timed callback function for sending periodic can signal.
        static void can_callback(rcl_timer_t *_timer, int64_t _last_call_time);
        #endif

        /// @brief Timed callback function for updating inverter motor speed.
        static void update_motor_speed_callback(rcl_timer_t *_timer, int64_t _last_call_time);

        /// @brief Timed callback function for testing control latency.
        static void latency_test_callback(rcl_timer_t *_timer, int64_t _last_call_time);

        /// @brief Callback function for receiving new messages form "/test_control_latency".
        static void onTestLatency(const void *_msg);
        
        /// @brief Function for testing control latency.
        static void latency_test_worker(void *_args);
};

extern MicroRosHandler micro_ros_handler;

#endif // MICRO_ROS_HANDLER_HPP
