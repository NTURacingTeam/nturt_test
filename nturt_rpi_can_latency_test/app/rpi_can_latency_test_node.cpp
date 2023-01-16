// std include
#include <memory>
#include <thread>

// ros2 include
#include "rclcpp/rclcpp.hpp"

// nturt include
#include "nturt_realtime_utils/memory_lock.hpp"
#include "nturt_realtime_utils/scheduling.hpp"
#include "nturt_rpi_can_latency_test/rpi_can_latency_test.hpp"

int main(int argc, char **argv) {
    lock_memory();
    set_thread_scheduling(pthread_self(), SCHED_FIFO, 80);
    
    rclcpp::init(argc, argv);
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    rclcpp::NodeOptions options;
    executor.add_node(std::make_shared<RpiCanLatencyTest>());
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
