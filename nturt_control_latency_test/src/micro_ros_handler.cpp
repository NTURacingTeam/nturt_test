#include "micro_ros_handler.hpp"

void MicroRosHandler::init() {
    // initialize gpios
    // built-in led pin
    pinMode(GPIO_NUM_2, OUTPUT);
    // dac pin (25 for DAC1, 26 for DAC2)
    pinMode(DAC1, OUTPUT);
    pinMode(DAC2, OUTPUT);
    // 1.12
    dacWrite(DAC1, 80);
    // 1.00
    dacWrite(DAC2, 88);

    // Serial.begin(119200);
    #ifndef MICRO_ROS_WIFI_TRANSPORT
    set_microros_serial_transports(Serial);
    delay(2000);
    #endif
    #ifdef MICRO_ROS_WIFI_TRANSPORT
    IPAddress agent_ip(172, 20, 10, 5);
    size_t agent_port = 8080;
    char ssid[] = "Ethereal";
    char passwd[]= "12345678";
    set_microros_wifi_transports(ssid, passwd, agent_ip, agent_port);
    #endif

    printf("Connected to micro ros.\n");

    // initialize can bus to baud rate of 500k and only accept id "0x0C0"
    if(!CAN.begin(500E3)) {
        error_loop(1, __LINE__, __func__);
    }
    CAN.filter(0x0C0);

    // initialize micro ros objects
    // initialize micro ros allocator object
    allocator_ = rcl_get_default_allocator();
    // initialize support object
    RCCHECK(rclc_support_init(&support_, 0, NULL, &allocator_));

    // initialize micro ros node
    RCCHECK(rclc_node_init_default(&node_, "micro_ros_node", "micro_ros", &support_));

    // initialize micro ros executor object, 5 things to be executed
    RCCHECK(rclc_executor_init(&executor_, &support_.context, 5, &allocator_));

    // initialize micro ros clock
    RCCHECK(rcl_ros_clock_init(&clock_, &allocator_));

    // intialize micro ros timer
    RCCHECK(rclc_timer_init_default(&ckeck_can_timer_, &support_, RCL_MS_TO_NS(1), check_can_callback));
    #ifdef SEND_INVERTER_SIGNAL
    RCCHECK(rclc_timer_init_default(&can_timer_, &support_, RCL_MS_TO_NS(5), can_callback));
    #endif
    RCCHECK(rclc_timer_init_default(&update_motor_speed_timer_, &support_, RCL_MS_TO_NS(100), update_motor_speed_callback));
    RCCHECK(rclc_timer_init_default(&latency_test_timer_, &support_, RCL_MS_TO_NS(2000), latency_test_callback));

    // create micro ros publisher
    #ifdef PUBLISH_RECEIVED_CAN
    RCCHECK(rclc_publisher_init_default(&can_pub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(can_msgs, msg, Frame), "/received_messages"));
    #endif
    #ifdef PUBLISH_MOTOR_SPEED
    RCCHECK(rclc_publisher_init_default(&motor_speed_pub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/motor_speed"));
    #endif
    RCCHECK(rclc_publisher_init_default(&latency_pub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/latency"));

    // create micro ros subscriber
    RCCHECK(rclc_subscription_init_default(&test_latency_sub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/test_control_latency"));

    // add micro ros timer to executor
    RCCHECK(rclc_executor_add_timer(&executor_, &ckeck_can_timer_));
    #ifdef SEND_INVERTER_SIGNAL
    RCCHECK(rclc_executor_add_timer(&executor_, &can_timer_));
    #endif
    RCCHECK(rclc_executor_add_timer(&executor_, &update_motor_speed_timer_));
    RCCHECK(rclc_executor_add_timer(&executor_, &latency_test_timer_));
    
    // immediately stop latency test timer until called
    RCCHECK(rcl_timer_cancel(&latency_test_timer_));

    // add micro ros subscriber to executer
    RCCHECK(rclc_executor_add_subscription(&executor_, &test_latency_sub_, &test_latency_msg_, &onTestLatency, ON_NEW_DATA));

    // initialize can time
    #ifdef SEND_INVERTER_SIGNAL
    int64_t this_time;
    RCSOFTCHECK(rcl_clock_get_now(&clock_, &this_time));
    last_call_.init(this_time);
    #endif
}

void MicroRosHandler::end() {
    // destroy micro ros objects
    RCCHECK(rcl_ros_clock_fini(&clock_));
    RCCHECK(rcl_timer_fini(&ckeck_can_timer_));
    #ifdef SEND_INVERTER_SIGNAL
    RCCHECK(rcl_timer_fini(&can_timer_));
    #endif
    RCCHECK(rcl_timer_fini(&update_motor_speed_timer_));
    #ifdef PUBLISH_RECEIVED_CAN
    RCCHECK(rcl_publisher_fini(&can_pub_, &node_));
    #endif
    #ifdef PUBLISH_MOTOR_SPEED
    RCCHECK(rcl_publisher_fini(&motor_speed_pub_, &node_));
    #endif
    RCCHECK(rcl_publisher_fini(&latency_pub_, &node_));
    RCCHECK(rcl_node_fini(&node_));
}

void MicroRosHandler::spin_once() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(10)));
}

void MicroRosHandler::error_loop(const rcl_ret_t _rc, const int _line, const char* _fun) {
    printf("Enter error loop with error code \"%d\" at line \"%d\" at function \"%s\"\n", _rc, _line, _fun);
    while(true) {
        for(int i = 0; i < 3; i++) {
            digitalWrite(GPIO_NUM_2, HIGH);
            usleep(200000);
            digitalWrite(GPIO_NUM_2, LOW);
            usleep(200000);
        }

        for(int i = 0; i < 3; i++) {
            digitalWrite(GPIO_NUM_2, HIGH);
            usleep(600000);
            digitalWrite(GPIO_NUM_2, LOW);
            usleep(200000);
        }

        for(int i = 0; i < 3; i++) {
            digitalWrite(GPIO_NUM_2, HIGH);
            usleep(200000);
            digitalWrite(GPIO_NUM_2, LOW);
            usleep(200000);
        }
    }

    usleep(1000000);
}

void MicroRosHandler::error_once(const rcl_ret_t _rc, const int _line, const char* _fun) {
    printf("Error once with error code \"%d\" at line \"%d\" at function \"%s\"\n", _rc, _line, _fun);
    digitalWrite(GPIO_NUM_2, HIGH);
    usleep(10000);
    digitalWrite(GPIO_NUM_2, LOW);
}

void MicroRosHandler::check_can_callback(rcl_timer_t *_timer, int64_t _last_call_time) {
    (void) _last_call_time;
    
    if(_timer != NULL) {
        // lock can
        std::lock_guard<std::mutex> lock(can_mutex_);

        if(CAN.parsePacket() != 0) {
            // get can data
            uint8_t raw_data[8] = {0};
            for(int i = 0; i < CAN.packetDlc(); i++) {
                raw_data[i] = CAN.read();
            }

            // decode can data, for how this work, please checkout nturt_can_parser
            // WARNNIG: assume torque_command is non-negtive
            torque_command_ = (raw_data[0] + (raw_data[1] << 8)) * 0.1;

            // publish to "/received_messages"
            #ifdef PUBLISH_RECEIVED_CAN
            int64_t now;
            RCSOFTCHECK(rcl_clock_get_now(&clock_, &now));
            can_msg_.header.stamp.sec = now / 1000000000;
            can_msg_.header.stamp.nanosec = now % 1000000000;
            can_msg_.id = CAN.packetId();
            can_msg_.is_rtr = CAN.packetRtr();
            can_msg_.is_extended = CAN.packetExtended();
            can_msg_.is_error = false;
            can_msg_.dlc = CAN.packetDlc();
            memcpy(can_msg_.data, raw_data, sizeof(raw_data));
            RCSOFTCHECK(rcl_publish(&can_pub_, &can_msg_, NULL));
            #endif
        }
    }
}

#ifdef SEND_INVERTER_SIGNAL
void MicroRosHandler::can_callback(rcl_timer_t *_timer, int64_t _last_call_time) {
    (void) _last_call_time;
    
    if(_timer != NULL) {
        int64_t this_time;
        RCSOFTCHECK(rcl_clock_get_now(&clock_, &this_time));
        
        // period 100ms
        if(this_time > last_call_.board_temp_ + RCL_MS_TO_NS(100)) {
            // lock can
            std::lock_guard<std::mutex> lock(can_mutex_);

            CAN.beginPacket(0x0A1);
            // control board temperature 60 [deg C]
            #ifndef RANDOM_INVERTER_SIGNAL
            CAN.write(88);
            CAN.write(4);
            #endif
            #ifdef RANDOM_INVERTER_SIGNAL
            CAN.write(this_time % 255);
            CAN.write(this_time % 128);
            #endif
            CAN.write(0);
            CAN.write(0);
            CAN.write(0);
            CAN.write(0);
            CAN.write(0);
            CAN.write(0);
            CAN.endPacket();

            last_call_.board_temp_ -= RCL_MS_TO_NS(100);
        }
        // period 100ms
        if(this_time > last_call_.motor_temp_ + RCL_MS_TO_NS(100)) {
            // lock can
            std::lock_guard<std::mutex> lock(can_mutex_);

            CAN.beginPacket(0x0A2);
            CAN.write(0);
            CAN.write(0);
            CAN.write(0);
            CAN.write(0);
            // motor temperature 60 [deg C]
            #ifndef RANDOM_INVERTER_SIGNAL
            CAN.write(88);
            CAN.write(4);
            #endif
            #ifdef RANDOM_INVERTER_SIGNAL
            CAN.write(this_time % 255);
            CAN.write(this_time % 128);
            #endif
            CAN.write(0);
            CAN.write(0);
            CAN.endPacket();

            last_call_.motor_temp_ -= RCL_MS_TO_NS(100);
        }
        // period 10ms
        if(this_time > last_call_.motor_speed_ + RCL_MS_TO_NS(10)) {
            // lock can
            std::lock_guard<std::mutex> lock(can_mutex_);

            CAN.beginPacket(0x0A5);
            CAN.write(0);
            CAN.write(0);
            // motor speed [rpm]
            // encode can data, for how this work, please checkout nturt_can_parser
            // WARNNING: it does NOT check for overflow
            CAN.write(static_cast<uint16_t>(motor_speed_) & 255);
            CAN.write((static_cast<uint16_t>(motor_speed_) >> 8) & 255);
            CAN.write(0);
            CAN.write(0);
            CAN.write(0);
            CAN.write(0);
            CAN.endPacket();

            last_call_.motor_speed_ -= RCL_MS_TO_NS(10);
        }
        // period 10ms
        if(this_time > last_call_.output_volt_ + RCL_MS_TO_NS(10)) {
            // lock can
            std::lock_guard<std::mutex> lock(can_mutex_);

            CAN.beginPacket(0x0A7);
            CAN.write(0);
            CAN.write(0);
            // output voltage 400 [V]
            #ifndef RANDOM_INVERTER_SIGNAL
            CAN.write(160);
            CAN.write(15);
            #endif
            #ifdef RANDOM_INVERTER_SIGNAL
            CAN.write(this_time % 255);
            CAN.write(this_time % 128);
            #endif
            CAN.write(0);
            CAN.write(0);
            CAN.write(0);
            CAN.write(0);
            CAN.endPacket();

            last_call_.output_volt_ -= RCL_MS_TO_NS(10);
        }
    }
}
#endif

void MicroRosHandler::update_motor_speed_callback(rcl_timer_t *_timer, int64_t _last_call_time) {
    (void) _last_call_time;
    
    if(_timer != NULL) {
        // motor speed is simply calculated as torque_command * 10
        #ifndef RANDOM_INVERTER_SIGNAL
        motor_speed_ = torque_command_ * 10;
        #endif
        #ifdef RANDOM_INVERTER_SIGNAL
        int64_t now;
        RCSOFTCHECK(rcl_clock_get_now(&clock_, &now));
        motor_speed_ = now % 999;
        #endif

        // publish to "/motor_speed"
        #ifdef PUBLISH_MOTOR_SPEED
        motor_speed_msg_.data = motor_speed_;
        RCSOFTCHECK(rcl_publish(&motor_speed_pub_, &motor_speed_msg_, NULL));
        #endif
    }
}

void MicroRosHandler::onTestLatency(const void *_msg) {
	const std_msgs__msg__Bool *msg = static_cast<const std_msgs__msg__Bool*>(_msg);
    printf("received latency request\n");
    
    if(msg->data && !begin_latency_test_) {
        printf("begin latency_test_worker\n");
        RCSOFTCHECK(rcl_timer_cancel(&update_motor_speed_timer_));
        motor_speed_ = 500;
        RCSOFTCHECK(rcl_timer_reset(&latency_test_timer_));
        begin_latency_test_ = true;
    }
    else if(!msg->data && begin_latency_test_) {
        printf("end latency_test_worker\n");
        RCSOFTCHECK(rcl_timer_cancel(&latency_test_timer_));
        RCSOFTCHECK(rcl_timer_reset(&update_motor_speed_timer_));
        begin_latency_test_ = false;

    }
}

void MicroRosHandler::latency_test_callback(rcl_timer_t *_timer, int64_t _last_call_time) {
    printf("run latency_test_worker\n");
    xTaskCreate(latency_test_worker, "latency_test_worker", 2048, NULL, 0, NULL);
}

void MicroRosHandler::latency_test_worker(void *_args) {
    (void) _args;

    float latency;
    rcl_time_point_value_t last_time, this_time;

    // 2.257
    dacWrite(DAC1, 185);
    // 2.473
    dacWrite(DAC2, 204);
    RCSOFTCHECK(rcl_clock_get_now(&clock_, &last_time));

    while(torque_command_ == 0) {
        usleep(500);
    }

    RCSOFTCHECK(rcl_clock_get_now(&clock_, &this_time));

    latency = (this_time - last_time) / 1E9;
    latency_msg_.data = latency;

    // 1.12
    dacWrite(DAC1, 80);
    // 1.00
    dacWrite(DAC2, 88);
    while(torque_command_ != 0) {
        usleep(500);
    }

    RCSOFTCHECK(rcl_publish(&latency_pub_, &latency_msg_, NULL));
    vTaskDelete(NULL);
}

// initialization of static variables (otherwise there will be compilation error)
std::mutex MicroRosHandler::can_mutex_;

rcl_allocator_t MicroRosHandler::allocator_;
rclc_support_t MicroRosHandler::support_;
rclc_executor_t MicroRosHandler::executor_;
rcl_node_t MicroRosHandler::node_;
rcl_clock_t MicroRosHandler::clock_;

rcl_timer_t MicroRosHandler::ckeck_can_timer_;
#ifdef SEND_INVERTER_SIGNAL
rcl_timer_t MicroRosHandler::can_timer_;
#endif
rcl_timer_t MicroRosHandler::update_motor_speed_timer_;
rcl_timer_t MicroRosHandler::latency_test_timer_;

#ifdef PUBLISH_RECEIVED_CAN
rcl_publisher_t MicroRosHandler::can_pub_;
#endif
#ifdef PUBLISH_MOTOR_SPEED
rcl_publisher_t MicroRosHandler::motor_speed_pub_;
#endif
rcl_publisher_t MicroRosHandler::latency_pub_;

rcl_subscription_t MicroRosHandler::test_latency_sub_;

#ifdef PUBLISH_RECEIVED_CAN
can_msgs__msg__Frame MicroRosHandler::can_msg_;
#endif
#ifdef PUBLISH_MOTOR_SPEED
std_msgs__msg__Float32 MicroRosHandler::motor_speed_msg_;
#endif
std_msgs__msg__Bool MicroRosHandler::test_latency_msg_;
std_msgs__msg__Float32 MicroRosHandler::latency_msg_;

#ifdef SEND_INVERTER_SIGNAL
LastCall MicroRosHandler::last_call_;
#endif
volatile float MicroRosHandler::torque_command_ = 0;
volatile float MicroRosHandler::motor_speed_ = 0;
volatile bool MicroRosHandler::begin_latency_test_ = false;

// create an instance of micro_ros_handle for use
extern MicroRosHandler micro_ros_handler;
