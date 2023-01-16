// glibc include
#include <pthread.h>
#include <sched.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

// std include
#include <functional>
#include <memory>

// ros2 include
#include "rclcpp/rclcpp.hpp"

// ros2 message include
#include "can_msgs/msg/frame.hpp"

// nturt include
#include "nturt_realtime_utils/memory_lock.hpp"
#include "nturt_realtime_utils/scheduling.hpp"

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for echoing can signal from "/to_can_bus" to "/from_can_bus", only change can id to "receive_id"
 * when receiving frame with can id "send_id".
 */
class FakeSocketCanBridgeNode : public rclcpp::Node {
    public:
        /// @brief Constructor of fake_socket_can_bridge_node.
        FakeSocketCanBridgeNode(rclcpp::NodeOptions _options) : Node("fake_socket_can_bridge_node", _options),
            can_pub_(this->create_publisher<can_msgs::msg::Frame>("/from_can_bus", 10)),
            can_sub_(this->create_subscription<can_msgs::msg::Frame>("/to_can_bus", 10, 
                std::bind(&FakeSocketCanBridgeNode::onCan, this, std::placeholders::_1))),
            
            send_id_(this->declare_parameter("send_id", 0x010)),
            receive_id_(this->declare_parameter("receive_id", 0x020)) {
        }

    private:
        /// @brief ROS2 publisher to "/from_can_bus", for sending fake received can signal to other nodes.
        rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;

        /// @brief ROS2 sbscriber to "/to_can_bus", for receiving can signal sending requirement from other nodes.
        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
        
        // internal states
        /// @brief Can id that the test can message will be sent.
        uint32_t send_id_;

        /// @brief Can id that will be received as the responding can test message.
        uint32_t receive_id_;

        /// @brief Callback function when receiving message from "/to_can_bus".
        void onCan(const can_msgs::msg::Frame &_msg) {
            if(_msg.id == send_id_) {
                auto msg = _msg;
                msg.id = receive_id_;
                can_pub_->publish(msg);
            }
            else {
                can_pub_->publish(_msg);
            }
        }
};

int main(int argc, char **argv) {
    // real-time configuration
    // lock_memory();
    // set_thread_scheduling(pthread_self(), SCHED_FIFO, 80);

    rclcpp::init(argc, argv);

    rclcpp::executors::StaticSingleThreadedExecutor executor;
    rclcpp::NodeOptions options;

    rclcpp::Node::SharedPtr fake_socket_can_bridge_node = std::make_shared<FakeSocketCanBridgeNode>(options);

    executor.add_node(fake_socket_can_bridge_node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
