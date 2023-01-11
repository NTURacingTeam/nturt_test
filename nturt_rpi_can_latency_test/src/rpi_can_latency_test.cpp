#include "nturt_rpi_can_latency_test/rpi_can_latency_test.hpp"

RpiCanLatencyTest::RpiCanLatencyTest() : Node("nturt_rpi_can_latency_test_node"),
    can_pub_(this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 10)),
    can_sub_(this->create_subscription<can_msgs::msg::Frame>("/from_can_bus", 10,
        std::bind(&RpiCanLatencyTest::onCan, this, std::placeholders::_1))) {

    // declear parameters
    this->declare_parameter("send_id", 0x010);
    this->declare_parameter("receive_id", 0x020);
    this->declare_parameter("is_echo_server", false);
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
    is_echo_server_ = this->get_parameter("is_echo_server").get_parameter_value().get<bool>();
    test_period_ = this->get_parameter("test_period").get_parameter_value().get<double>();
    test_length_ = this->get_parameter("test_length").get_parameter_value().get<double>();
    std::string logging_file_name = this->get_parameter("logging_file_name").get_parameter_value().get<std::string>();

    if(!is_echo_server_) {
        RCLCPP_INFO(this->get_logger(), "The test will be running for %fs at %fs per test message.", test_length_, test_period_);

        // register onShutdown to ros
        rclcpp::on_shutdown(std::bind(&RpiCanLatencyTest::onShutdown, this));

        // open csv_file
        std::string file = ament_index_cpp::get_package_share_directory("nturt_rpi_can_latency_test") + "/" + logging_file_name;
        RCLCPP_INFO(this->get_logger(), "Log file at: %s", file.c_str());
        csv_file_.open(file, std::ios::out | std::ios::trunc);

        // csv headers
        csv_file_ << "frame_count" << ',' << "latency[s]" << std::endl;

        // allocate memory for wirte_buffer_
        write_buffer_ = std::make_unique<char[]>(20 * test_length_ / test_period_);

        // start timer
        starting_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&RpiCanLatencyTest::starting_callback, this));
        RCLCPP_INFO(this->get_logger(), "Wait a second for everything to initialize before starting the test...");

        program_start_time_ = this->now().seconds();
    }
    else {
        RCLCPP_INFO(this->get_logger(), "This test is configured as echo server. Ctrl + C to stop.");
    }
}

void RpiCanLatencyTest::onShutdown() {
    csv_file_ << write_buffer_.get();
    csv_file_.close();
}

void RpiCanLatencyTest::onCan(const can_msgs::msg::Frame &_msg) {
    if(!is_echo_server_ && _msg.id == receive_id_) {
        double now = this->now().seconds();
        FrameConversion compose;
        compose.frame_data_ = _msg.data;
        int frame_count = compose.information.frame_count_;
        float latency = static_cast<float>(now - program_start_time_) - compose.information.time_stemp_;

        // log to write_buffer_
        write_buffer_index_ += sprintf(write_buffer_.get() + sizeof(char) * write_buffer_index_, "%d,%f\n", frame_count, latency);
    }
    else if(is_echo_server_ && _msg.id == send_id_) {
        can_msgs::msg::Frame msg = _msg;
        msg.id = receive_id_;
        can_pub_->publish(msg);
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

    // compose frame data to contain frame counts and sent time
    FrameConversion compose;
    compose.information.frame_count_ = frame_count_++;
    compose.information.time_stemp_ = static_cast<float>(this->now().seconds() - program_start_time_);
    frame.data = compose.frame_data_;
    
    can_pub_->publish(frame);
}

void RpiCanLatencyTest::starting_callback() {
    starting_timer_->cancel();
    can_latency_test_timer_ = this->create_wall_timer(std::chrono::duration<double>(test_period_),
        std::bind(&RpiCanLatencyTest::can_latency_test_callback, this));
    stopping_timer_ = this->create_wall_timer(std::chrono::duration<double>(test_length_),
            std::bind(&RpiCanLatencyTest::stopping_callback, this));
    RCLCPP_INFO(this->get_logger(), "RPI can latency test starts now.");
}

void RpiCanLatencyTest::stopping_callback() {
    can_latency_test_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Rpi can latency test complete, waiting 1 second for not yet arrived frames...");
    rclcpp::sleep_for(1s);
    
    RCLCPP_INFO(this->get_logger(), "Test completed, shutting down ros.");
    rclcpp::shutdown();
}
