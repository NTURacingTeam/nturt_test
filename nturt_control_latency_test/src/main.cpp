#include "micro_ros_handler.hpp"

void setup() {
    micro_ros_handler.init();
}

void loop() {
    micro_ros_handler.spin_once();
}
