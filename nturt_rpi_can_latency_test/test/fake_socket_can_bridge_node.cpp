// std include
#include <functional>
#include <memory>
#include <string.h>

// ros2 include
#include "rclcpp/rclcpp.hpp"

// ros2 message include
#include "can_msgs/msg/frame.hpp"

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for echoing can signal from "/to_can_bus" to "/from_can_bus", only change can id to "receive_id"
 * when receiving frame with can id "send_id".
 */
class FakeSocketCanBridgeNode : public rclcpp::Node {
    public:
        /// @brief Constructor of fake_socket_can_bridge_node.
        FakeSocketCanBridgeNode() : Node("fake_socket_can_bridge_node"),
            can_pub_(this->create_publisher<can_msgs::msg::Frame>("/from_can_bus", 10)),
            can_sub_(this->create_subscription<can_msgs::msg::Frame>("/to_can_bus", 10, 
                std::bind(&FakeSocketCanBridgeNode::onCan, this, std::placeholders::_1))) {
        
            // declear parameters
            this->declare_parameter("send_id", 0x010);
            this->declare_parameter("receive_id", 0x020);

            // get parameters
            send_id_ = this->get_parameter("send_id").get_parameter_value().get<uint32_t>();
            receive_id_ = this->get_parameter("receive_id").get_parameter_value().get<uint32_t>();
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
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeSocketCanBridgeNode>());
    rclcpp::shutdown();

    return 0;
}
