# NTURT RPI CAN Latency Test

## Introduction

This ROS2 package is used for determining the latency inherent to ROS2 node -- ROS2 transport layer -- ros2_socketcan (replacement of socket_can_bridge in ROS1) -- socketCAN -- CAN HAT. In order to properly pin point the latency source of the controll system.

This test is basically a `round trip test`, where the message will first be sent as described above, and then sent back in similar order. If neglecting the latency of the response side (where the exact same message is sent back to rpi, and hopely the respond side is at low latency such as a microcontroller), the latency can be calculated by dividing the total latency by two.

## Usage

### Running the test

The test can be run by using the launch file `nturt_rpi_can_latency_test.launch.py` by

```bash=
ros2 launch nturt_rpi_can_latency_test nturt_rpi_can_latency_test.launch.py
```

### Launch Configuration

There are couple launch parameters defined in the `nturt_rpi_can_latency_test.launch.py` launch file:

1. `is_realtime` - bool: Whether to run the test in realtime mode, default tp `true`.
2. `using_fake_socket_can_bridge` - bool: Whether to use fake socket_can_bridge (only echoes the can messages sent from the test node back to it self), default to `false`.
3. `send_id` - unsigned int: The can id that the can test message will be sent [^1], default to `0x010`.
4. `receive_id` - unsigned int: The can id that will be received as the responding can test message [^1], default to `0x020`.
5. `is_echo_server` - bool: Whether this test is run as a echo server that will not perform the test, but rather echo can frames with id equals to `send_id` back, with id equals to `receive_id`, default to `false`.
6. `test_period` - double: Period between each test can signals are sent [s], default to `0.01`.
7. `test_length` - double: How long the test will run [s], default to `60.0`.
8. `logging_file_name` - string: The file name that the test data file will be [^2](note that it outputs a `csv` file), default to `<current_time>.csv`.

[^1]: In order to avoid potential problems, the send_id and receive_id should be different.

[^2]: The test data file will be at the the share directory of this package, namely `<path_to_workspace>/install/nturt_rpi_can_latency_test/`.

### Plotting Tool

There's also a plotting tool available for visualizing the test data, it can be run by

```bash=
ros2 run nturt_rpi_can_latency_test plot_test_result.py
```

## Test Result

### Test Subject

The test is performed on Raspberry Pi 4 B+ with 2 Gb RAM with realtime kernel adopted from [ros-realtime/ros-realtime-rpi4-image](https://github.com/ros-realtime/ros-realtime-rpi4-imag).

### Test Type

There are two options that total make up four different tests:

1. `Real-time` - whether the test is conducted under realtime condition (specifically memory lock and elevated priority).
2. `Stress` - whether the test is run under cpu load (specifically using `stress-ng -c 4` command).

### CAN Latency Round Trip Test

The results of the rpi can bus round trip test between two rpi are shown here:

The test can signals for normal test (not labeled as `fast`) were sent at the rate of one per 10 ms and fast tests were at the rate of one per 2 ms. The tests are length of 1 min.

#### Non-realtime

![](doc/can_rtt/non-realtime.png "Non-realtime")

#### Non-realtime, Stressed

![](doc/can_rtt/non-realtime_stressed.png "Non-realtime, Stressed")

#### Non-realtime, Fast

![](doc/can_rtt/non-realtime_fast.png "Non-ealtime, Fast, Stressed")

#### Non-realtime, Fast, Stressed

Unfortunely, the test result is either only partially logged to the test result file or not logged at all, so there's no result for this test.

#### Realtime

![](doc/can_rtt/realtime.png "Realtime")

#### Realtime, Stressed

![](doc/can_rtt/realtime_stressed.png "Realtime, Stressed")

#### Realtime, Fast

![](doc/can_rtt/realtime_fast.png "Realtime, Fast")

#### Realtime, Fast, Stressed

![](doc/can_rtt/realtime_fast_stressed.png "Realtime, Fast, Stressed")

#### Other discoveries

- The socket can is quite demanding that the receiver takes 20% of a rpi 4 core and sender takes 15% at a rate of one can signal per 2 ms.
- Futher increase the can singal rate to one per 1 ms will leads to dramatic increase of latency to around 400 ms and overflow of can bus buffer.

### ROS2 Message Round Trip Test

By setting `using_fake_socket_can_bridge` launch parameter to true, this test can also be used as a round trip test for ros2 message on a singal rpi, the results are shown here:

The test messages were sent at the rate of one per 1 ms with a test length of 1 min, and the payload size is 56 bytes.

#### Non-realtime[^3]

![](doc/message_rtt/non-realtime.png "Non-realtime")

#### Non-realtime, Stressed[^3]

![](doc/message_rtt/non-realtime_stressed.png "Non-realtime, Stressed")

#### Realtime[^3]

![](doc/message_rtt/realtime.png "Realtime")

#### Realtime, Stressed[^3]

![](doc/message_rtt/realtime_stressed.png "Realtime, Stressed")

[^3]: The title of the figures are wrong, but I'm too lazy to change them.

## Conclusion

- For some reason, whether the process is realtime or not does not effect the can latency reoud trip test, which still has to be further studied, but is it clear that it has major impact on ros2 message round trip test.
- When under cpu load, the average latency increased while large jitter happens less, which may due to the power saving feature of the cpu.
- It is obvious that larger jitter still exist in realtime configuration, which may resulted from the DDS layer that still dynamically allocate memory though ROS2 layer is memory allocation free.
