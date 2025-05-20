# pi5_UDP

This ROS 2 Python package handles **UDP communication** for the Raspberry Pi 5 (Pi5), intended to send control messages to the Teensy microcontroller over a wired Ethernet connection. It includes test and production nodes to simulate and handle message transmission.

---

## Table of Contents

- [Overview](#overview)
- [Package Nodes](#package-nodes)
- [Build Instructions](#build-instructions)
- [Run Instructions](#run-instructions)
- [Current Architecture](#current-architecture)
- [TO DO](#to-do)

---

## Overview

This package provides:

- A simulated node (`pseudo_auto_mode`) to generate test control strings.
- A UDP sender node (`udp_sender`) that listens for these strings and transmits them over UDP.
- A UDP receiver node (`udp_receiver`) for debugging/testing the received messages on the Teensy or another system.

All messages follow the format:

```
throttle,steering_angle,emergency_flag
```

- `throttle`: Float to 2 decimal places (0.00 to 100.00)
- `steering_angle`: Float to 2 decimal places (-50.00 to 50.00)
- `emergency_flag`: Integer `0` or `1` (1 every 4th loop for test purposes)

---

## Package Nodes

### `pseudo_auto_mode.py`

- Publishes test strings to the `/auto_commands` topic.
- Increments `throttle` and `steering_angle` cyclically.
- Sets emergency flag to `1` every 4th loop.

### `udp_sender.py`

- Subscribes to `/auto_commands`
- Sends messages over UDP to a predefined IP and port.

### `udp_receiver.py`

- Listens on UDP port 5005 and logs received messages.
- Useful for debugging and verifying messages sent from the Pi5.

---

## Build Instructions

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

---

## Run Instructions

From the workspace root, after sourcing:

```bash
ros2 launch pi5_UDP pi5_UDP.launch.py
```

This will start:
- `pseudo_auto_mode`
- `udp_sender`
- `udp_receiver`

To run any node manually:

```bash
ros2 run pi5_UDP pseudo_auto_mode
ros2 run pi5_UDP udp_sender
ros2 run pi5_UDP udp_receiver
```

To verify messages:

```bash
ros2 topic echo /auto_commands
```

---

## Current Architecture

```text
[pseudo_auto_mode] --> /auto_commands --> [udp_sender] --UDP--> [receiver on Teensy or debug system]
```

- All nodes log activity to screen using `rclpy` logging.
- Pseudo auto mode runs in test setups only (future plans include conditional execution).
- Default IP and port are hardcoded in `udp_sender.py`.

---

## TO DO

- [ ] **Move package to Raspberry Pi 5 and test over physical Ethernet.**
- [ ] **Set correct IP address in `udp_sender.py` to match Teensy's wired IP.**
- [ ] **Ensure Teensy is listening on port `5005` and receiving correctly.**
- [ ] **Modify `pseudo_auto_mode.py` to only run in test/dev mode (via launch argument or debug flag).**
- [ ] **Create a real `auto_command_publisher` node (from high-level stack) to publish real messages.**
- [ ] [Optional] Add ability to switch between pseudo/real modes in a launch file using arguments.
- [ ] [Optional] Add configuration file or parameters to manage IP/port instead of hardcoding.

---

This package is under active development for integration into the EVT MINI-ROB autonomous vehicle control stack.
