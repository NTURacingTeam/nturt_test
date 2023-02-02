#include "microros_latency_test.h"

// libc include
#include <stdint.h>

// stm32 include
#include <cmsis_os.h>

// microros include
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>

#include "microros_utils.h"

// nturt include
#include "nturt_ros_interface/msg/latency_test_message.h"
#include "nturt_ros_interface/msg/single_latency_test_result.h"

/// @brief Micro ros allocator object.
static rcl_allocator_t allocator;

/// @brief Micro ros support object.
static rclc_support_t support;

/// @brief Micro ros executor object.
static rclc_executor_t executor;

/// @brief Micro ros node.
static rcl_node_t node;

/// @brief Micro ros clock.
static rcl_clock_t clock;

/// @brief Micro ros timer for periodically testing micro ros latency.
static rcl_timer_t micro_ros_latency_test_timer;

/// @brief Micro ros timer for periodically testing micro ros latency.
static rcl_timer_t freertos_states_timer;

/// @brief Micro ros publisher to "/to_host" for publishing test message to
/// host.
static rcl_publisher_t test_message_pub;

/// @brief Micro ros publisher to "/single_test_result" for publishing single
/// test result.
static rcl_publisher_t test_result_pub;

/// @brief Micro ros publisher to "/freertos_stats" for publishing freertos
/// stats.
static rcl_publisher_t freertos_stats_pub;

/// @brief Micro ros subscriber to "/from_host" for subscribing test message
/// from host.
static rcl_subscription_t test_message_sub;

/// @brief Micro ros message "nturt_ros_interface/msg/LatencyTestMessage"
/// for publishing.
static nturt_ros_interface__msg__LatencyTestMessage pub_latency_test_msg;

/// @brief Micro ros message "nturt_ros_interface/msg/LatencyTestMessage"
/// for subscribing.
static nturt_ros_interface__msg__LatencyTestMessage sub_latency_test_msg;

/// @brief Micro ros message
/// "nturt_ros_interface/msg/SingleLatencyTestResult" for publishing single test
/// result.
static nturt_ros_interface__msg__SingleLatencyTestResult
    single_latency_test_result_msg;

static char freertos_stats_msg_data[1024];

/// @brief Micro ros message "std_msgs/msg/String" for publishing freertos
/// stats.
static std_msgs__msg__String freertos_stats_msg = {
    .data.data = freertos_stats_msg_data,
    .data.size = 0,
    .data.capacity = 1024,
};

// internal states
/// @brief How many times the test messages were sent.
static int message_count = 0;

/// @brief Timed callback function to publish test message.
static void micro_ros_latency_test_callback(rcl_timer_t *_timer,
                                            int64_t _last_call_time);

/// @brief Timed callback function to publish freertos stats.
static void freertos_stats_callback(rcl_timer_t *_timer,
                                    int64_t _last_call_time);

/// @brief Callback function for receiving message form "/from_host".
static void onTestMessage(const void *_msg);

void start_microros_latency_test_task(void *_argument) {
  (void)_argument;

  configure_microros();

  // initialize micro ros objects
  // initialize micro ros allocator object
  allocator = rcl_get_default_allocator();
  // initialize support object
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // initialize micro ros node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_latency_test_node", "",
                                 &support));

  // initialize micro ros executor object, 3 things to be executed
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));

  // initialize micro ros clock
  RCCHECK(rcl_ros_clock_init(&clock, &allocator));

  // intialize micro ros timer
  RCCHECK(rclc_timer_init_default(&micro_ros_latency_test_timer, &support,
                                  RCL_MS_TO_NS(100),
                                  micro_ros_latency_test_callback));
  RCCHECK(rclc_timer_init_default(&freertos_states_timer, &support,
                                  RCL_MS_TO_NS(1000),
                                  freertos_stats_callback));

  // create micro ros publisher
  RCCHECK(rclc_publisher_init_default(
      &test_message_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nturt_ros_interface, msg, LatencyTestMessage),
      "/to_host"));
  RCCHECK(rclc_publisher_init_default(
      &test_result_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nturt_ros_interface, msg,
                                  SingleLatencyTestResult),
      "/single_test_result"));
  RCCHECK(rclc_publisher_init_default(
      &freertos_stats_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/freertos_stats"));

  // create micro ros subscriber
  RCCHECK(rclc_subscription_init_default(
      &test_message_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nturt_ros_interface, msg, LatencyTestMessage),
      "/from_host"));

  // add micro ros timer to executor
  RCCHECK(rclc_executor_add_timer(&executor, &micro_ros_latency_test_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &freertos_states_timer));

  // add micro ros subscriber to executer
  RCCHECK(rclc_executor_add_subscription(&executor, &test_message_sub,
                                         &sub_latency_test_msg, &onTestMessage,
                                         ON_NEW_DATA));

  while (1) {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
    osDelay(10);
  }
}

static void micro_ros_latency_test_callback(rcl_timer_t *_timer,
                                            int64_t _last_call_time) {
  (void)_last_call_time;

  if (_timer != NULL) {
    int64_t now;
    RCSOFTCHECK(rcl_clock_get_now(&clock, &now));

    pub_latency_test_msg.time_stemp = now;
    pub_latency_test_msg.count = message_count++;

    RCSOFTCHECK(rcl_publish(&test_message_pub, &pub_latency_test_msg, NULL));
  }
}

static void freertos_stats_callback(rcl_timer_t *_timer,
                                    int64_t _last_call_time) {
  (void)_last_call_time;

  if (_timer != NULL) {
    float runtime_percent;
    UBaseType_t uxArraySize = uxTaskGetNumberOfTasks();
    TaskStatus_t *pxTaskStatusArray =
        pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));

    if (pxTaskStatusArray != NULL) {
      freertos_stats_msg.data.size = 0;
      unsigned long ulTotalRunTime;
      uxArraySize =
          uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);

      freertos_stats_msg.data.size +=
          sprintf(freertos_stats_msg.data.data + freertos_stats_msg.data.size,
                  "Task count = %lu\n", uxArraySize);
      freertos_stats_msg.data.size +=
          sprintf(freertos_stats_msg.data.data + freertos_stats_msg.data.size,
                  "No      Name          S Usage   HW\n");

      for (int i = 0; i < uxArraySize; i++) {
        runtime_percent =
            (float)(100 * (float)pxTaskStatusArray[i].ulRunTimeCounter /
                    (float)ulTotalRunTime);

        freertos_stats_msg.data.size += sprintf(
            freertos_stats_msg.data.data + freertos_stats_msg.data.size,
            "Task %d: %-12s %2d %6.3f%% %4d\n", i,
            pxTaskStatusArray[i].pcTaskName, pxTaskStatusArray[i].eCurrentState,
            runtime_percent, pxTaskStatusArray[i].usStackHighWaterMark);
      }

      // does not count for null termination charactor
      freertos_stats_msg.data.size--;
      RCSOFTCHECK(rcl_publish(&freertos_stats_pub, &freertos_stats_msg, NULL));

      vPortFree(pxTaskStatusArray);
    } else {
      RCSOFTCHECK(RCL_RET_BAD_ALLOC);
    }
  }
}

static void onTestMessage(const void *_msg) {
  const nturt_ros_interface__msg__LatencyTestMessage *msg =
      (const nturt_ros_interface__msg__LatencyTestMessage *)_msg;

  int64_t now;
  RCSOFTCHECK(rcl_clock_get_now(&clock, &now));

  single_latency_test_result_msg.count = msg->count;
  single_latency_test_result_msg.latency =
      (now - msg->time_stemp) / 1000000000.0;

  RCSOFTCHECK(
      rcl_publish(&test_result_pub, &single_latency_test_result_msg, NULL));
}
