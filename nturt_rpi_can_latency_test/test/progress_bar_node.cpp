// glibc include
#include <stdio.h>

// std include
#include <functional>
#include <memory>
#include <string>

// ros2 include
#include "rclcpp/rclcpp.hpp"

static constexpr int bar_length = 50;
static constexpr char reset[] = "\033[0m";
static constexpr char green[] = "\033[32m";
static constexpr char blue[] = "\033[34m";

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for testing printing progress bar in ros2.
 */
class ProgressBar : public rclcpp::Node {
    public:
        /// @brief Constructor of fake_socket_can_bridge_node.
        ProgressBar(rclcpp::NodeOptions _options) : Node("fake_socket_can_bridge_node", _options),
            progress_bar_timer_(this->create_wall_timer(std::chrono::milliseconds(100),
                std::bind(&ProgressBar::progress_bar_callback, this))) {

        }

    private:
        /// @brief ROS2 timer for updating the progress bar.
        rclcpp::TimerBase::SharedPtr progress_bar_timer_;

        // internal states
        /// @brief Counter counting the total progress.
        int total_progress_ = 0;

        /// @brief Counter counting the pritial progress.
        int partial_progress_ = 0;

        /// @brief Timed callback function for updating the progress bar.
        void progress_bar_callback() {
            int green_bar_length = static_cast<int>(bar_length * partial_progress_ / 100);
            int blue_bar_length = static_cast<int>(bar_length * (total_progress_ - partial_progress_) / 100);
            int empty_bar_length = bar_length - green_bar_length - blue_bar_length;
            std::cout << "progress: ["
                << green << std::string(green_bar_length, '|')
                << blue << std::string(blue_bar_length, '|')
                << reset << std::string(empty_bar_length, ' ')
                << ']' << total_progress_ << "%\r";
            std::cout.flush();

            if(total_progress_ == 100) {
                std::cout << std::endl;
                rclcpp::shutdown();
            }

            total_progress_++;
            if(total_progress_ % 2 == 0) {
                partial_progress_++;
            }
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    rclcpp::executors::StaticSingleThreadedExecutor executor;
    rclcpp::NodeOptions options;

    rclcpp::Node::SharedPtr progress_bar_node = std::make_shared<ProgressBar>(options);

    executor.add_node(progress_bar_node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
