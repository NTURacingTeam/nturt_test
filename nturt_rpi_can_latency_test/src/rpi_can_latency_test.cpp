#include "nturt_rpi_can_latency_test/rpi_can_latency_test.hpp"

RpiCanLatencyTest::RpiCanLatencyTest() : Node("nturt_rpi_can_latency_test_node"),
    can_pub_(this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 10)),
    can_sub_(this->create_subscription<can_msgs::msg::Frame>("/from_can_bus", 10, std::bind(&RpiCanLatencyTest::onCan, this, std::placeholders::_1))) {

    // declear parameters
    this->declare_parameter("send_id", 0x010);
    this->declare_parameter("receive_id", 0x020);
    this->declare_parameter("test_period", 0.1);
    this->declare_parameter("test_length", 60.0);
    // default logging file name to "current_time.csv"
    char default_file_name[100];
    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    strftime(default_file_name, sizeof(default_file_name), "%Y-%m-%d-%H-%M-%S.csv", std::localtime(&now));
    this->declare_parameter("logging_file_name", default_file_name);

    // get parameters
    send_id_ = this->get_parameter("send_id").get_parameter_value().get<uint32_t>();
    receive_id_ = this->get_parameter("receive_id").get_parameter_value().get<uint32_t>();
    double test_period = this->get_parameter("test_period").get_parameter_value().get<double>();
    double test_length = this->get_parameter("test_length").get_parameter_value().get<double>();
    std::string logging_file_name = this->get_parameter("logging_file_name").get_parameter_value().get<std::string>();

    // start timer
    can_latency_test_timer_ = this->create_wall_timer(std::chrono::duration<double>(test_period),
        std::bind(&RpiCanLatencyTest::can_latency_test_callback, this));
    stopping_timer_ = this->create_wall_timer(std::chrono::duration<double>(test_length),
        std::bind(&RpiCanLatencyTest::stopping_callback, this));
    RCLCPP_INFO(this->get_logger(), "The test will be running for %fs at %fs per test message.", test_length, test_period);

    // register onShutdown to ros
    rclcpp::on_shutdown(std::bind(&RpiCanLatencyTest::onShutdown, this));

    // open csv_file
    std::string file = ament_index_cpp::get_package_share_directory("nturt_rpi_can_latency_test") + "/" + logging_file_name;
    RCLCPP_INFO(this->get_logger(), "Log file at: %s", file.c_str());
    csv_file_.open(file + logging_file_name, std::ios::out | std::ios::trunc);

    // csv headers
    csv_file_ << "frame_count" << ',' << "latency[s]" << std::endl;

    program_start_time_ = this->now().seconds();
}

void RpiCanLatencyTest::onShutdown() {

    csv_file_.close();
}

void RpiCanLatencyTest::onCan(const std::shared_ptr<can_msgs::msg::Frame> _msg) {
    double now = this->now().seconds();
    if(_msg->id == receive_id_) {
        FrameConversion compose;
        compose.frame_data_ = _msg->data;
        int frame_count = compose.information.frame_count_;
        float latency = static_cast<float>(now - program_start_time_) - compose.information.time_stemp_;

        // log to csv
        csv_file_ << frame_count << ',' << latency << std::endl;
    }
}

void RpiCanLatencyTest::can_latency_test_callback() {
    //test frame with 8 bytes in length
    can_msgs::msg::Frame frame;
    frame.id = send_id_;
    frame.dlc = 8;
    frame.is_error = false;
    frame.is_extended = false;
    frame.is_rtr = false;

    // compose frame data to contain frame count and sent time
    FrameConversion compose;
    compose.information.frame_count_ = frame_count_++;
    compose.information.time_stemp_ = static_cast<float>(this->now().seconds() - program_start_time_);
    frame.data = compose.frame_data_;
    
    can_pub_->publish(frame);
}

void RpiCanLatencyTest::stopping_callback() {
    can_latency_test_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Rpi can latency test complete, waiting 1 second for not yet arrived frames...");
    rclcpp::sleep_for(1s);
    
    RCLCPP_INFO(this->get_logger(), "Test completed, shutting down ros.");
    rclcpp::shutdown();
}