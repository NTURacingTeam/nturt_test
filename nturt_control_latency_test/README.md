# NTURT latency test

## Introduction

In order to test the round trip latency from actuating accelerator pedel signal to rpi mcu command can signal, we utilize an esp32 micro controller in platform io arduino core with micro_ros as the software structure to setup such latency test.

This software also acts as a testing fake inverter that sends some portion of the can signal as `CM200` from cascadia would, including `motor_speed`, `motor_temperature`, `control_board_temperature`, `output_voltage`.


> Note: This package is implemented in ros2 instead of ros 1 we used in other parts of the nturt electrical system. And this software was only test in `humble` release. If you were interested in using it, using `ros2_host` image from [NTURacingTeam/docker](https://github.com/NTURacingTeam/docker) is a great place to start.

## Usage

### Test setup

#### CAN

A can tranceiver should be used to send/receive can signal, where here `can_tx` should be connected to pin 5, `can_rx` should be connected to pin 4 as the default of `CAN` arduino library.

#### DAC

Both dacs of the esp32 is used in order to mimic the signal behavior of the two accelerator pedal sensors. However, as for the configuration of `EP4`, the signal is pulled up, hence the esp32 dac have to drain the current to pull down the volatge to 1~2.7 V of typical legal accelerator pedal sensor signal, which is (maybe) not the intended use of such esp32, and it behaves oddly. As a result a voltage follower constructed by a op-amp is used to reach the goal.

### Configuring the node

There are some compile time configurations that can be set to change the behavior of this software, which are

1. SEND_INVERTER_SIGNAL: When defined, send mimic inverter signal to can bus.
2. MICRO_ROS_WIFI_TRANSPORT: When defined, uses wifi as the trasport to micro_ros, please checkout: [mirco_ros/Overview](https://micro.ros.org/docs/tutorials/advanced/overview/). Otherwise, serial of baudrate `119200` is used instead. And don't forget to switch the `board_microros_transport` config in `platformio.ini` accordingly.
3. PUBLISH_RECEIVED_CAN: When defind, sends received inverter command of id `0X0C0` to topic `received_messages` in ros message format `can_msgs/Frame`.
4. PUBLISH_MOTOR_SPEED: When defined, sends fake motor speed to topic `/motor_speed` in `std_msgs/Bool` ros message format
5. RANDOM_INVERTER_SIGNAL: When defined, sends time-varying inverter signal instead of the constant hard-coded in the sftware.

### Running the node

First compile the code and upload it to esp32, then run micro_ros_agent on the host machine to monitor the message returned from the microcontroller.

### Measuring the latency

By sending true to topic `/test_control_latency` in `std_msgs/Bool` ros message format, latency can be measured and the result can be found in topic `/latency` in `std_msgs/Float32` ros message format.

## Bug, pitfall and pain

Yes, really pain.

### Bug

For some reason, action does not work and it always returns error when initialize a action server.

### Pitfall

1. It seemed that the amount of timer that can be created is bounded by the harware timer of the esp32 (4 to be exact). Any more will result in error.
2. For some reason, if a publisher or a message object is initialized without every called, other publisher/subscriber will not work.
3. Receiving hundreds of can signal and publish them to micro ros topic is too heavy for esp32, so a filter is necessary to keep the load low.
