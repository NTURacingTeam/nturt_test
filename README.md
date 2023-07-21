# NTURT Test

## Introduction

This project aims to test the control system.

## Tests Available

Integration tests:

- [nturt_control_integration_test](nturt_control_integration_test/README.md)

Latency tests:

- [nturt_control_latency_test](nturt_control_latency_test/README.md), result: roughly 60 ms
- [nturt_micro_ros_latency_test](nturt_micro_ros_latency_test/README.md), result: up to 2 ms, but consumes 99% of cpu time
- [nturt_rpi_can_latency_test](nturt_rpi_can_latency_test/README.md), result: roughly 3 ms for two rpi
