# pi5_udp

This ROS 2 package handles **UDP communication** for the Raspberry Pi 5. It sends autonomous driving commands to a Teensy microcontroller over a wired Ethernet connection and receives telemetry back. It also includes a test mode using simulated input.

---

## Table of Contents

- [Overview](#overview)
- [Nodes](#nodes)
- [Architecture](#architecture)
- [Build Instructions](#build-instructions)
- [Run Instructions](#run-instructions)
- [To Do](#to-do)

---

## Overview

This package includes:

- `pseudo_auto_mode`: Generates simulated driving commands for testing.
- `udp_sender`: Sends commands from `/auto_commands` to the Teensy over UDP.
- `udp_receiver`: Listens for UDP telemetry messages and publishes them to `/teensy_telemetry`.

All nodes use ROS 2 (`rclpy`) and are configured to be used in launch files or individually.

---

## Nodes

### `pseudo_auto_mode`

- Publishes strings like `"50.00,30.00,0"` to `/auto_commands`.
- Loops through test values:
  - `throttle`: 0 to 100 (step 5)
  - `steering_angle`: -50 to 50 (step 10)
  - `emergency_flag`: 1 every 4th message

### `udp_sender`

- Subscribes to `/auto_commands`
- Sends each message via UDP to a **hardcoded IP and port** (update this before deploying).

### `udp_receiver`

- Listens on UDP port `5005`
- Parses strings like:
  ```
  "AUTO,540.2,23.5,49.1,13.2,5.8,3.4,0.8"
  ```
- Publishes parsed data to the `/teensy_telemetry` topic using the `TeensyTelemetry` message from the `system_msgs` package.

---

## Architecture

```
[pseudo_auto_mode]
        ↓
/auto_commands  ─────────→  [udp_sender]  ─── UDP ───→  Teensy

                                         ↑
                        [udp_receiver] ←─ UDP  ←─  Teensy
                                         ↓
                              /teensy_telemetry
```

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

To launch all nodes:

```bash
ros2 launch pi_teensy_udp pi_teensy_udp.launch.py
```

To run nodes individually:

```bash
ros2 run pi5_udp pseudo_auto_mode
ros2 run pi5_udp udp_sender
ros2 run pi5_udp udp_receiver
```

To test if messages are flowing:

```bash
ros2 topic echo /auto_commands
ros2 topic echo /teensy_telemetry
```

---

## To Do

- [ ] Move package to Pi5 and test with live Teensy via Ethernet
- [ ] Update IP in `udp_sender.py` to match Teensy IP (e.g., `192.168.0.177`)
- [ ] Add parameterization (IP/port as ROS params or YAML config)
- [ ] Add launch argument to toggle `pseudo_auto_mode` on/off
- [ ] Replace `pseudo_auto_mode` with high-level stack node in production
- [ ] Add optional logging or bag recording for telemetry

---

## Status

This package is functional and used for real-time UDP communication between the Pi5 and Teensy as part of the EVT MINI-ROB autonomous vehicle stack. Further development is planned to generalize and extend test/control capabilities.
