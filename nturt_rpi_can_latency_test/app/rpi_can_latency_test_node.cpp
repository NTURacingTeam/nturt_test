// std include
#include <list>
#include <memory>
#include <thread>

// ros2 include
#include "rclcpp/rclcpp.hpp"

// nturt include
#include "nturt_realtime_utils/memory_lock.hpp"
#include "nturt_realtime_utils/scheduling.hpp"
#include "nturt_rpi_can_latency_test/rpi_can_latency_test.hpp"

static const std::list<std::string> realtime_keys = {
    "realtime",
    "real-time",
    "real_time",
    "--realtime",
    "--real-time",
    "--real_time",
};

int main(int argc, char **argv) {
    // real-time configuration
    if (argc > 1) {
        for (auto & realtime_key : realtime_keys) {
            if (std::string(argv[1]) == realtime_key) {
                lock_memory();
                set_thread_scheduling(pthread_self(), SCHED_FIFO, 80);
                break;
            }
        }
    }

    rclcpp::init(argc, argv);

    rclcpp::executors::StaticSingleThreadedExecutor executor;
    rclcpp::NodeOptions options;

    rclcpp::Node::SharedPtr rpi_can_latency_test_node = std::make_shared<RpiCanLatencyTest>(options);

    executor.add_node(rpi_can_latency_test_node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
