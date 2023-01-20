#include "nturt_rpi_can_latency_test/rpi_can_latency_test.hpp"

static constexpr int bar_length = 50;
static constexpr char reset[] = "\033[0m";
static constexpr char green[] = "\033[32m";
static constexpr char blue[] = "\033[34m";

RpiCanLatencyTest::RpiCanLatencyTest(rclcpp::NodeOptions _options) : Node("nturt_rpi_can_latency_test_node", _options),
    can_pub_(this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 10)),
    can_sub_(this->create_subscription<can_msgs::msg::Frame>("/from_can_bus", 10,
        std::bind(&RpiCanLatencyTest::onCan, this, std::placeholders::_1))),
    can_latency_test_timer_(this->create_wall_timer(std::chrono::duration<double>(this->declare_parameter("test_period", 0.01)),
        std::bind(&RpiCanLatencyTest::can_latency_test_callback, this))),
    starting_timer_(this->create_wall_timer(std::chrono::seconds(1),
        std::bind(&RpiCanLatencyTest::starting_callback, this))),
    progress_timer_(this->create_wall_timer(std::chrono::seconds(1),
        std::bind(&RpiCanLatencyTest::progress_callback, this))),

    terminal_("/dev/tty"),

    send_id_(this->declare_parameter("send_id", 0x010)),
    receive_id_(this->declare_parameter("receive_id", 0x020)),
    is_echo_server_(this->declare_parameter("is_echo_server", false)),
    test_length_(this->declare_parameter("test_length", 60.0)) {

    // cancel timer
    can_latency_test_timer_->cancel();
    starting_timer_->cancel();
    
    if(!is_echo_server_) {
        starting_timer_->call();

        // get parameter values that's already declared
        test_period_ = this->get_parameter("test_period").get_parameter_value().get<double>();

        total_count_ = static_cast<int>(test_length_ / test_period_);

        // default logging file name to "current_time.csv"
        char default_file_name[100];
        std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        strftime(default_file_name, sizeof(default_file_name), "%Y-%m-%d-%H-%M-%S.csv", std::localtime(&now));
        std::string logging_file_name = this->declare_parameter("logging_file_name", default_file_name);

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
        RCLCPP_INFO(this->get_logger(), "Wait a second for everything to initialize before starting the test...");
        starting_timer_->reset();

        program_start_time_ = this->now().seconds();
    }
    else {
        RCLCPP_INFO(this->get_logger(), "This test is configured as echo server. Ctrl + C to stop.");
    }
}

void RpiCanLatencyTest::onShutdown() {
    progress_timer_->execute_callback();
    terminal_ << std::endl;

    csv_file_ << write_buffer_.get();
    csv_file_.close();
}

void RpiCanLatencyTest::onCan(const can_msgs::msg::Frame &_msg) {
    received_count_++;

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
    compose.information.frame_count_ = sent_count_;
    compose.information.time_stemp_ = static_cast<float>(this->now().seconds() - program_start_time_);
    frame.data = compose.frame_data_;

    can_pub_->publish(frame);

    sent_count_++;
    if(sent_count_ == total_count_) {
        can_latency_test_timer_->cancel();
        progress_timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "Rpi can latency test complete, waiting 1 second for not yet arrived frames...");
        rclcpp::sleep_for(1s);

        RCLCPP_INFO(this->get_logger(), "Test completed, shutting down ros.");
        rclcpp::shutdown();
    }
}

void RpiCanLatencyTest::starting_callback() {
    starting_timer_->cancel();

    RCLCPP_INFO(this->get_logger(), "RPI can latency test starts now.");
    can_latency_test_timer_->reset();
    progress_timer_->reset();
}

void RpiCanLatencyTest::progress_callback() {
    if(!is_echo_server_) {
        int green_bar_length = static_cast<int>(bar_length * received_count_ / total_count_);
        int blue_bar_length = static_cast<int>(bar_length * (sent_count_ - received_count_) / total_count_);
        int empty_bar_length = bar_length - green_bar_length - blue_bar_length;
        terminal_ << std::fixed << "progress: ["
            << green << std::string(green_bar_length, '|')
            << blue << std::string(blue_bar_length, '|')
            << reset << std::string(empty_bar_length, ' ')
            << "] sent: "
            << blue << sent_count_ << '(' << std::setprecision(1) << 100.0 * sent_count_ / total_count_ << "%)"
            << reset << " received: "
            << green << received_count_ << '(' << std::setprecision(1) << 100.0 * received_count_ / total_count_ << "%)"
            << reset << " total: " << total_count_ << " receive rate: " << std::setprecision(5) << 100.0 * received_count_ / sent_count_ << "%\r";
        terminal_.flush();
    }
    else {
        terminal_ << std::fixed << "Received: " << std::setprecision(5) << static_cast<double>(received_count_ - last_received_count_)
            << " during last second.\r";
        terminal_.flush();
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(RpiCanLatencyTest)
