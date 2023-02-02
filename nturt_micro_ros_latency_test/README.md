# NTURT Micro ROS Latency Test

## Introduction

This ROS2/micro ROS package us used for determining the latency inherent to ROS2 node -- micro ROS agent -- physical transport layer(specifically uart in this test) -- micro ROS node. In order to evaluate the possibilities of running micro ROS as the commuication layer for microcontrollers.

This test is basically a `round trip test`, where the message will first be sent as described above, and then sent back in similar order. If neglecting the latency of the response side (where the exact same message is sent back to microcontroller, and hopely the respond side is at low latency[^1]), the latency can be calculated by dividing the total latency by two.

[^1]: In this case the response side is actually a pc, but in latter section, we will find that the bottleneck is not here.

## Usage

### Microcontroller

This test is run on `nucleo-h743zi2` and it's base on stm32cube ide environment with [micro-ROS/micro_ros_stm32cubemx_utils](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils) as the micro ros frame work.

#### Build and flash

Before build, add

```yaml=
nturt_ros_interfaces:
  type: git
  url: https://github.com/NTURacingTeam/nturt_ros_interface
  version: ros2
```

to `nturt_micro_ros_latency_test/nturt_micro_ros_latency_test_microcontroller/micro_ros_stm32cubemx_utils/microros_static_library_ide/extra_packages/extra_packages.repo`.(since for modified content of a submodule can not be added to git)

The build and flash process is the same as the standard workflow of using stm32cude ide, with custon include/library path for micro ros all taken care of, you just need to push the magical `run` button on the stm32cude ide.

### Host

#### Micro ros agent

Micro ros requires [micro-ROS/micro-ROS-Agent](https://github.com/micro-ROS/micro-ROS-Agent) on the host to communicate with other ros2 nodes running on the host mechine, which can be run by

```shell=
ros2 run micro_ros_agent micro_ros_agent serial -b 460800 --dev /dev/ttyACM0
```

which will use uart with baudrate 460800 to communicate with stm32.

#### Echo server

Run the echo server on the host side by

```shell=
ros2 run nturt_micro_ros_latency_test_host nturt_micro_ros_latency_test_host_node
```

> Note: Since this test is hardly replicable due to prior knowledges and hardware constrains, there will be no more utilities to visualize the test result.

## Test result

After correctly setting everything up, the latency can be monitored on ros topic `/single_test_result` for every single test, and the cpu usage can be monitored on `/freertos_stats` every second.

### No OS

The test was originally conducted in bare metal environment, and the latency is listed below:

- 8ms at uart baudrate of 115200 and test publish rate of 100hz
- 2ms at uart baudrate of 460800 and test publish rate of 100hz

The test message is 12 bytes long and the result message is 8 bytes long for total of 32 bytes per test, which results in around 3 KB/s of data transfer.

At higher test rate, the latency becomes latger and also more unstable.(system reset, missing messages, etc)

Since micro ros was developed under the assumption of some underlying rtos and the test program is written in `platformio` environment, which is all non-ideal, the code will not be included here.

### Wtih FreeRTOS

The test result in freertos architecture is listed below:

- 2ms and cpu usage >99% at uart baudrate of 460800 with osDelay(1)[^2] and test publish rate of 10hz
- 7ms and cpu usage ~60% t uart baudrate of 460800 with osDelay(5)[^2] and test publish rate of 10hz
- Latency is roughly x + 2 ms for osDelay(x)[^2] and the cpu usage also drops significantly, but result in more unstable behaviors

Also, when updating the cpu usage to `/freertos_stats`, the latency increased by 30ms, and drop back to normal.

[^2]: This means that each micro ros spin is delayed by x ms for freertos to perform other tasks where x is in osDelay(x).

### Other findings

- The clock in [micro-ROS/micro-ROS-Agent](https://github.com/micro-ROS/micro-ROS-Agent) has only 1 ms precision due to using the time source from freertos.
- With freertos and stm32cube ide environment, micro ros becomes very unstable even in slow test publishing rate(10ms), which keeps stopping publishing new messages.

## Conclusion

- It seem that micro ros for stm32 is more unstable that in platformio environment, or this may due to an extra layer of os.
- CPU usage of micro ros spin is too high even no callback has to be executed. Given that stm32h7 series is currently some of the most power microcontrollers, it renders micro ros useless in tight real-time applications.
- UART baudrate have significant impact on latency, yet transmitting 32 bytes in baudrate of 115200 only takes 0.8ms (or a little bit more countin stop bit).
